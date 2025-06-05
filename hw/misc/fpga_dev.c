#include "qemu/osdep.h"
#include "hw/pci/pci_device.h"
#include "hw/pci/msi.h"
#include "hw/qdev-properties.h"
#include "qemu/event_notifier.h"
#include "qemu/module.h"
#include "system/kvm.h"
#include "qom/object.h"
#include <linux/if.h>
#include <linux/if_tun.h>
#include <sys/ioctl.h>
#include <fcntl.h>


#define TYPE_FPGA_DEV "fpga-dev"

enum {
    /* register offsets */
    Cmd        = 0x0,
    RxRingAddr = 0x4,
    TxRingAddr = 0x8,
    IntrMask   = 0xc,
    IntrStatus = 0x10,
    RegAddrMax = IntrStatus,

    /* Command register */
    CmdReset    = (1 << 2),  /* Enable to reset; self-clearing */
    RxOn        = (1 << 1),  /* Rx mode enable */
    TxOn        = (1 << 0),  /* Tx mode enable */

    /* IntrMask / IntrStatus registers */
    RxEmpty     = (1 << 4),  /* No Rx descriptors available */
    TxErr       = (1 << 3),  /* Tx error */
    TxOK        = (1 << 2),  /* Tx packet sent */
    RxErr       = (1 << 1),  /* Rx error */
    RxOK        = (1 << 0),  /* Rx packet received */

    /* Tx and Rx status descriptors */
    DescOwn     = (1 << 31), /* Descriptor is owned by FPGA */
    RingEnd     = (1 << 30), /* End of descriptor ring */
    TxError     = (1 << 23), /* Tx error summary */
    RxError     = (1 << 20), /* Rx error summary */
};

#define FpgaReg(offset) (s->regs[(offset)/4])

#define RX_BD_ADDR(index) (FpgaReg(RxRingAddr)+(index)*16)
#define TX_BD_ADDR(index) (FpgaReg(TxRingAddr)+(index)*16)

/* Rx/Tx Desc */
struct buffer_desc {
    uint32_t opts1;/* status */
    uint32_t opts2;/* for future extension */
    uint64_t addr;
} __attribute__((packed));


#define BUFFER_FULL (1<<0)
#define BUFFER_EMPTY (1<<1)

#define N 64

typedef struct FpgaDevState {
    PCIDevice parent_obj;
    MemoryRegion mmio; // 内存映射 I/O 区域
                       
    int tap_fd;        // TAP device file descriptor
    char *tap_name;    // TAP device name (command-line parameter)
    QemuMutex socket_mutex;

    uint32_t rx_tail;
    uint32_t rx_ring_size;

    uint32_t tx_tail;
    uint32_t tx_ring_size;
 
    QemuThread rx_thread;
    QemuThread tx_thread;
    QemuMutex thr_mutex;
    QemuCond thr_cond;

    uint32_t msi_enabled;

    uint32_t rx_seq;
    uint32_t tx_seq;
    QemuMutex fpga_mutex;
    uint32_t regs[16];
} FpgaDevState;


#define FPGA_DEV(obj) \
    OBJECT_CHECK(FpgaDevState, (obj), TYPE_FPGA_DEV)

static void handle_reset(FpgaDevState *s) {
    uint32_t seq;

    /* disable rx/tx */
    qemu_mutex_lock(&s->fpga_mutex);
    s->regs[Cmd] &= ~(RxOn|TxOn);
    qemu_mutex_unlock(&s->fpga_mutex);

    seq = s->rx_seq;
    /* wait rx thread to complete */
    while (seq == s->rx_seq) {
        usleep(1);
    }

    seq = s->tx_seq;
    /* wait tx thread to complete */
    while (seq == s->tx_seq) {
        usleep(1);
    }
}

static void handle_cmd(FpgaDevState *s, uint64_t value) {
    if (value & CmdReset) {

        /* reset all resources */
        qemu_mutex_lock(&s->fpga_mutex);
        s->regs[Cmd/4] &= ~(RxOn|TxOn);
        qemu_mutex_unlock(&s->fpga_mutex);

        /* wait rx/tx complete */
        handle_reset(s);

        s->rx_tail = 0;
        s->tx_tail = 0;

        /* reset done */
        qemu_mutex_lock(&s->fpga_mutex);
        s->regs[RxRingAddr/4] = 0;
        s->regs[TxRingAddr/4] = 0;
        s->regs[IntrMask/4] = 0;
        s->regs[IntrStatus/4] = 0;
        s->regs[Cmd/4] = 0; /* It should be the last one */
        qemu_mutex_unlock(&s->fpga_mutex);
    }
}

/*
 * @brief handle IntrStatus write operation
 *        Once some bit is set to '1', that means to reset it
 */
static void handle_status(FpgaDevState *s, uint64_t value) {
    qemu_mutex_lock(&s->fpga_mutex);
    s->regs[IntrStatus/4] &= ~value;
    qemu_mutex_unlock(&s->fpga_mutex);
}

typedef void (*handler)(FpgaDevState *s, uint64_t value);

typedef struct _FpgaHandler {
    hwaddr addr;
    handler fn;
} FpgaHandler_t;

FpgaHandler_t handlers[] = {
    {Cmd, handle_cmd},
    {IntrStatus, handle_status},
};

/*
 * @brief guest device operation in mainloop thread
 * @param opaque The pointer of FpgaDevState structure variable
 * @param addr The address of control regs
 * @param value The value written to the @addr
 * @param size The operation width of the @value
 * @return void
 */
static void fpga_dev_write(void *opaque, hwaddr addr, uint64_t value, unsigned size) {
    FpgaDevState *s = FPGA_DEV(opaque);
    int i;

    //printf("Write to addr 0x%lx, value 0x%lx, size %u\n", addr, value, size);

    // addr overflow
    if (addr > RegAddrMax) {
        fprintf(stderr, "fpga_dev_write: addr overflow\n");
        return;
    }

    // validate written size of value
    if (size!=1 && size!=2 && size != 4) {
        fprintf(stderr, "fpga_dev_write: unsupported size: %d\n", size);
        return;
    }

    qemu_mutex_lock(&s->fpga_mutex);

    switch (size) {
        case 1:
            s->regs[addr/4] |= (value&0xff);
            break;
        case 2:
            s->regs[addr/4] |= (value&0xffff);
            break;
        case 4:
            s->regs[addr/4] |= value&0xffffffff;
            break;
        default:
            break;
    }

    qemu_mutex_unlock(&s->fpga_mutex);

    for (i=0; i<sizeof(handlers)/sizeof(FpgaHandler_t); i++)
    {
        if (handlers[i].addr == addr)
            handlers[i].fn(s, value);
    }
}

/*
 * @brief guest device operation in mainloop thread
 * @param opaque The pointer of FpgaDevState structure variable
 * @param addr The address of control regs
 * @param value The value written to the @addr
 * @param size The operation width of the @value
 * @return uint64_t Return value of the @addr of the device
 */
static uint64_t fpga_dev_read(void *opaque, hwaddr addr, unsigned size) {
    FpgaDevState *s = FPGA_DEV(opaque);
    uint64_t value = 0;

    // addr overflow
    if (addr > RegAddrMax) {
        fprintf(stderr, "fpga_dev_read: addr overflow\n");
        return 0;
    }

    // validate written size of value
    if (size!=1 && size!=2 && size != 4) {
        fprintf(stderr, "fpga_dev_read: unsupported size: %d\n", size);
        return 0;
    }

    qemu_mutex_lock(&s->fpga_mutex);

    value = s->regs[addr/4];

    qemu_mutex_unlock(&s->fpga_mutex);

    switch(size) {
        case 1:
            value &=0xff;
            break;
        case 2:
            value &=0xffff;
            break;
        case 4:
            value &=0xffffffff;
            break;
        default:
            break;
    }

    //printf("Read from addr 0x%lx, value 0x%lx, size %u\n", addr, value, size);

    return value;
}

static const MemoryRegionOps fpga_dev_ops = {
    .read = fpga_dev_read,
    .write = fpga_dev_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void host_mem_read(FpgaDevState *s, uint64_t addr, uint8_t *buf, uint32_t size) {
    MemTxAttrs attrs = { .memory = true }; // 默认内存属性
                                           
    //printf("Read host mem from addr 0x%lx, len 0x%x\n", addr, size);
    address_space_rw(&address_space_memory, addr, attrs, buf, size, false);

}

static void host_mem_write(FpgaDevState *s, uint64_t addr, uint8_t *buf, uint32_t size) {
    MemTxAttrs attrs = { .memory = true }; // 默认内存属性
                                           //
    //printf("Write host mem from addr 0x%lx, len 0x%x\n", addr, size);
    address_space_rw(&address_space_memory, addr, attrs, buf, size, true);

}

static void buffer_desc_read(FpgaDevState *s, uint64_t addr, struct buffer_desc *bd) {

    MemTxAttrs attrs = { .memory = true }; // 默认内存属性
                                           
    address_space_rw(&address_space_memory, addr, attrs, bd, sizeof(*bd), false);

    //printf("Read buffer desc from addr 0x%lx\n", addr);
}

static void buffer_desc_write(FpgaDevState *s, uint64_t addr, struct buffer_desc *bd) {
    MemTxAttrs attrs = { .memory = true }; // 默认内存属性
                                           
    address_space_rw(&address_space_memory, addr, attrs, bd, sizeof(bd), true);

    //printf("Write buffer desc to addr 0x%lx\n", addr);
}

static void fpga_trigger_interrupt(FpgaDevState *s)
{
    PCIDevice *pci_dev = PCI_DEVICE(s);

    if (msi_enabled(pci_dev)) {
        msi_notify(pci_dev, 0); /* 触发第 0 个中断向量 */
    } else {
        /* 回退到传统中断（INTx） */
        pci_set_irq(pci_dev, 1);
    }
}

/*
 * @brief process one DMA operation for rx
 */
static void handle_rx(FpgaDevState *s) {

    static uint8_t buffer[2048];
    uint32_t addr; /* buffer_desc address */
    struct buffer_desc bd;
    uint32_t status;
    uint32_t status_reply; /* for update buffer status */
    uint64_t data_addr;
    int32_t len;

    qemu_mutex_lock(&s->fpga_mutex);
    addr = RX_BD_ADDR(s->rx_tail); /* indirect fpga regs accessing */
    qemu_mutex_unlock(&s->fpga_mutex);

    /* read current buffer desc */
    buffer_desc_read(s, addr, &bd);
    status = bd.opts1;

    /* 
     * buffer desc owned by sw
     * wait available buffers
     */
    if (!(status&DescOwn)) {
        /* no more buffer desc */

        qemu_mutex_lock(&s->fpga_mutex);
        if (FpgaReg(IntrMask) & RxEmpty) {
            FpgaReg(IntrStatus) |= RxEmpty;
            qemu_mutex_unlock(&s->fpga_mutex);

            fpga_trigger_interrupt(s);
        } else {
            qemu_mutex_unlock(&s->fpga_mutex);
        }

        return;
    }

    data_addr = bd.addr;

    status_reply = 0;
    /*
     * receive packet from tap device
     */
    qemu_mutex_lock(&s->fpga_mutex);
    len = read(s->tap_fd, buffer, sizeof(buffer));
    qemu_mutex_unlock(&s->fpga_mutex);
    if (len > 0) {
        host_mem_write(s, data_addr, buffer, len);

        status_reply |= (len&0x1fff);
    } else {
        if (errno==EAGAIN)
        {
            /* read return with no packet now */
            return;
        }

        status_reply |= RxError;
    }

    /* update status of buffer desc and writeback it */
    status_reply &= ~DescOwn;
    bd.opts1 = status_reply; 
    buffer_desc_write(s, addr, &bd);

    /* interrupt notify */
    qemu_mutex_lock(&s->fpga_mutex);
    if (FpgaReg(IntrMask) & (RxOK|RxErr)) {

        if (status_reply&RxError)
            FpgaReg(IntrStatus) |= RxErr;
        else
            FpgaReg(IntrStatus) |= RxOK;
        qemu_mutex_unlock(&s->fpga_mutex);

        fpga_trigger_interrupt(s);
    } else {
        /* irq has been disabled */
        qemu_mutex_unlock(&s->fpga_mutex);
    }

    /* update rx tail pointer */
    if (status&RingEnd)
        s->rx_tail = 0;
    else
        s->rx_tail++;
}


static void *fpga_rx_thread(void *opaque) {
    FpgaDevState *s = opaque;

    //printf("fpga rx thread is running...\n");

    while (1) {
        qemu_mutex_lock(&s->fpga_mutex);

        if (FpgaReg(Cmd) & RxOn) {
            qemu_mutex_unlock(&s->fpga_mutex);
            handle_rx(s);

            usleep(1000);

            continue;
        }

        qemu_mutex_unlock(&s->fpga_mutex);

        /* update sequence lock for fpga reset command */
        s->rx_seq++;

        /* make cpu happy */
        usleep(1000);
    }

    return NULL;
}

static void handle_tx(FpgaDevState *s) {
    static uint8_t buffer[2048];/* TBD: Is it big enough? */
    uint32_t addr; /* buffer_desc address */
    struct buffer_desc bd;
    uint32_t status;
    uint32_t status_reply; /* for update buffer status */
    uint64_t data_addr;
    int32_t len;

    qemu_mutex_lock(&s->fpga_mutex);
    addr = TX_BD_ADDR(s->tx_tail); /* indirect fpga regs accessing */
    qemu_mutex_unlock(&s->fpga_mutex);

    /* read current buffer desc */
    buffer_desc_read(s, addr, &bd);
    status = bd.opts1;

    /* 
     * buffer desc owned by sw
     * wait for available buffers
     */
    if (!(status&DescOwn)) {
        /* no more buffer desc */
        return;
    }

    data_addr = bd.addr;
    status = bd.opts1;
    len = status &0x1fff;
    //printf("tx: slot=%d, len=%d\n", s->tx_tail, len);

    /*
     * DMA
     */
    host_mem_read(s, data_addr, (uint8_t*)buffer, len);
#if 0
    printf("dump pkt:");
    for (i=0;i<len;i++) {
        if (i%16==0)
            printf("\n");
        printf("%02x ", buffer[i]);
    }
    printf("\n");
#endif


    status_reply = 0;
    /*
     * send packet to tap device
     */
    qemu_mutex_lock(&s->socket_mutex);
    len = write(s->tap_fd, buffer, len);
    //printf("tx return:%d\n", len);
    qemu_mutex_unlock(&s->socket_mutex);
    if (len <= 0) {
        status_reply |= TxError;
    }

    /* update status of buffer desc and writeback it */
    status_reply &= ~DescOwn;
    bd.opts1 = status_reply; 
    buffer_desc_write(s, addr, &bd);

    /* interrupt notify */
    qemu_mutex_lock(&s->fpga_mutex);
    if (FpgaReg(IntrMask) & (TxOK|TxErr)) {

        if (status_reply&TxError)
            FpgaReg(IntrStatus) |= TxErr;
        else
            FpgaReg(IntrStatus) |= TxOK;
        qemu_mutex_unlock(&s->fpga_mutex);

        fpga_trigger_interrupt(s);
    } else {
        /* irq has been disabled */
        qemu_mutex_unlock(&s->fpga_mutex);
    }

    /* update rx tail pointer */
    if (status&RingEnd)
        s->tx_tail = 0;
    else
        s->tx_tail++;
}

static void *fpga_tx_thread(void *opaque) {
    FpgaDevState *s = opaque;

    //printf("fpga tx thread is running...\n");

    while (1) {
        qemu_mutex_lock(&s->fpga_mutex);

        if (FpgaReg(Cmd) & TxOn) {
            qemu_mutex_unlock(&s->fpga_mutex);
            handle_tx(s);

            usleep(1000);

            continue;
        }

        qemu_mutex_unlock(&s->fpga_mutex);

        /* update sequence lock for fpga reset command */
        s->tx_seq++;

        /* make cpu happy */
        usleep(1000);
    }

    return NULL;
}

// Property definitions
static const Property fpga_properties[] = {
    DEFINE_PROP_STRING("tap-name", FpgaDevState, tap_name),
};

static int tun_alloc(char *dev, int flags) {
    struct ifreq ifr;
    int fd, err;

    if ((fd = open("/dev/net/tun", O_RDWR)) < 0) {
        perror("Opening /dev/net/tun");
        return fd;
    }

    memset(&ifr, 0, sizeof(ifr));
    ifr.ifr_flags = flags;
    if (*dev) {
        strncpy(ifr.ifr_name, dev, IFNAMSIZ);
    }

    if ((err = ioctl(fd, TUNSETIFF, (void *)&ifr)) < 0) {
        perror("ioctl(TUNSETIFF)");
        close(fd);
        return err;
    }

    strcpy(dev, ifr.ifr_name);
    return fd;
}

#if 0
static void delete_tap(const char *dev) {
    char cmd[256];
    snprintf(cmd, sizeof(cmd), "ip link set %s down", dev);
    system(cmd);
    snprintf(cmd, sizeof(cmd), "ip tuntap del mode tap name %s", dev);
    system(cmd);
}
#endif

static int set_nonblock(int fd) {
    // Get current flags
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags == -1) {
        perror("fcntl F_GETFL");
        return -1;
    }

    // Add O_NONBLOCK flag
    flags |= O_NONBLOCK;

    // Set new flags
    if (fcntl(fd, F_SETFL, flags) == -1) {
        perror("fcntl F_SETFL");
        return -1;
    }

    return 0;
}

static int tap_initailize(FpgaDevState *s, Error **errp) {
    char cmd[256];
    // Use tap-name from command line, default to "tap0" if not specified
    if (!s->tap_name) {
        s->tap_name = g_strdup("tap0");
    }

    // Open TAP device
    s->tap_fd = tun_alloc(s->tap_name, IFF_TAP | IFF_NO_PI);
    if (s->tap_fd < 0) {
        error_setg(errp, "Failed to open TAP device %s", s->tap_name);
        g_free(s->tap_name);
        return -1;
    }

    set_nonblock(s->tap_fd);

    // Bring up TAP interface
    snprintf(cmd, sizeof(cmd), "ip link set %s up", s->tap_name);
    if (system(cmd) != 0) {
        error_setg(errp, "Failed to bring up TAP device %s", s->tap_name);
        close(s->tap_fd);
        g_free(s->tap_name);
        return -1;
    }

    return 0;
}

static void fpga_dev_realize(PCIDevice *pci_dev, Error **errp) {
    PCIDeviceClass *pc = PCI_DEVICE_GET_CLASS(pci_dev);
    FpgaDevState *s = FPGA_DEV(pci_dev);

    // 初始化 MMIO 区域
    memory_region_init_io(&s->mmio, OBJECT(s), &fpga_dev_ops, s, "fpga_dev-mmio", 0x100);

    // 注册 BAR0（MMIO）
    pci_register_bar(pci_dev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->mmio);

    // 设置 PCIe 设备属性
    pci_config_set_vendor_id(pci_dev->config, pc->vendor_id);
    pci_config_set_device_id(pci_dev->config, pc->device_id);
    pci_config_set_revision(pci_dev->config, pc->revision);
    pci_config_set_class(pci_dev->config, pc->class_id);

    /* 初始化 MSI，支持 1 个中断向量 */
    if (msi_init(pci_dev, 0x0, 1, true, false, errp)) {
        error_setg(errp, "Failed to initialize MSI");
        return;
    }

    if (tap_initailize(s, errp)<0) {
        error_setg(errp, "Failed to initialize tap");
        return;
    }

    /*
     * BUG:
     * Right now, msi has not been enabled by the driver
     */
    s->msi_enabled = msi_enabled(pci_dev);

    // Initialize mutex
    qemu_mutex_init(&s->fpga_mutex);
    qemu_mutex_init(&s->socket_mutex);

    // 初始化寄存器值

    qemu_thread_create(&s->rx_thread, "fpga-rx", fpga_rx_thread,
            s, QEMU_THREAD_JOINABLE);

    qemu_thread_create(&s->tx_thread, "fpga-tx", fpga_tx_thread,
            s, QEMU_THREAD_JOINABLE);
}

static void fpga_dev_class_init(ObjectClass *klass, const void *data) {
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *pc = PCI_DEVICE_CLASS(klass);

    dc->desc = "FPGA Device";

    pc->realize = fpga_dev_realize;
    pc->vendor_id = 0x1234;
    pc->device_id = 0x5678;
    pc->class_id = PCI_CLASS_OTHERS;

    // Register properties
    device_class_set_props(dc, fpga_properties);
}

static const TypeInfo fpga_dev_info = {
    .name = TYPE_FPGA_DEV,
    .parent = TYPE_PCI_DEVICE,
    .instance_size = sizeof(FpgaDevState),
    .class_init = fpga_dev_class_init,
    .interfaces = (const InterfaceInfo[]) {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { },
    },

};

static void fpga_dev_register_types(void) {
    type_register_static(&fpga_dev_info);
}

type_init(fpga_dev_register_types)
