#![no_std]
#![no_main]

use defmt::*;
use panic_probe as _;

use cyw43_pio::{DEFAULT_CLOCK_DIVIDER, PioSpi};

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, PIO0, USB};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_time::{Duration, Timer};

use embassy_rp::usb as rp_usb;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State as CdcState};
use embassy_usb::Builder;

use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::pipe::Pipe;

use embedded_io_async::Write;

use static_cell::StaticCell;

mod usb_defmt_logger {
    use core::sync::atomic::{AtomicBool, AtomicPtr, Ordering};
    use core::ptr;
    use embassy_sync::blocking_mutex::raw::NoopRawMutex;
    use embassy_sync::pipe::Pipe;
    use static_cell::StaticCell;

    pub static PIPE_CELL: StaticCell<Pipe<NoopRawMutex, 4096>> = StaticCell::new();
    static PIPE_PTR: AtomicPtr<Pipe<NoopRawMutex, 4096>> = AtomicPtr::new(ptr::null_mut());

    pub fn init_pipe() -> &'static Pipe<NoopRawMutex, 4096> {
        let pipe = PIPE_CELL.init(Pipe::new());
        PIPE_PTR.store(pipe as *const _ as *mut _, Ordering::Release);
        pipe
    }

    #[defmt::global_logger]
    struct Logger;

    static TAKEN: AtomicBool = AtomicBool::new(false);

    unsafe impl defmt::Logger for Logger {
        fn acquire() {
            while TAKEN.swap(true, Ordering::Acquire) {}
        }

        unsafe fn release() {
            TAKEN.store(false, Ordering::Release);
        }

        unsafe fn flush() {}

        unsafe fn write(bytes: &[u8]) {
            let p = PIPE_PTR.load(Ordering::Acquire);
            if p.is_null() {
                return;
            }
            let pipe = unsafe { &*p };
            let mut s = bytes;
            while !s.is_empty() {
                match pipe.try_write(s) {
                    Ok(n) => s = &s[n..],
                    Err(_) => break,
                }
            }
        }
    }
}

// Program metadata for `picotool info`
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"Blinky Example with defmt"),
    embassy_rp::binary_info::rp_program_description!(c"Let's blink and chill."),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

// interrupt types are available via https://docs.embassy.dev/embassy-rp/git/rp2040/interrupt/typelevel/index.html
bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    USBCTRL_IRQ => rp_usb::InterruptHandler<USB>;
});

#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>,
) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn usb_dev_task(mut device: embassy_usb::UsbDevice<'static, rp_usb::Driver<'static, USB>>) -> ! {
    device.run().await
}

#[embassy_executor::task]
async fn usb_log_task(
    mut tx: embassy_usb::class::cdc_acm::Sender<'static, rp_usb::Driver<'static, USB>>,
    pipe: &'static Pipe<NoopRawMutex, 4096>,
) -> ! {
    tx.wait_connection().await;

    let mut chunk = [0u8; 64];
    loop {
        let n = pipe.read(&mut chunk).await;
        if n == 0 { continue; }
        let _ = tx.write_all(&chunk[..n]).await;
        if n == 64 {
            let _ = tx.write_all(&[]).await;
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let pipe = usb_defmt_logger::init_pipe();

    let driver = rp_usb::Driver::new(p.USB, Irqs);

    static USB_CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    static USB_BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    static USB_MSOS_DESCRIPTOR: StaticCell<[u8; 128]> = StaticCell::new();
    static USB_CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();

    let cfg_desc = USB_CONFIG_DESCRIPTOR.init([0; 256]);
    let bos_desc = USB_BOS_DESCRIPTOR.init([0; 256]);
    let msos_desc = USB_MSOS_DESCRIPTOR.init([0; 128]);
    let ctrl_desc = USB_CONTROL_BUF.init([0; 64]);

    let mut usb_cfg = embassy_usb::Config::new(0x2e8a, 0x000a);
    usb_cfg.manufacturer = Some("Pico 2 W");
    usb_cfg.product = Some("defmt over USB CDC");
    usb_cfg.serial_number = Some("0001");
    usb_cfg.max_power = 100;
    usb_cfg.max_packet_size_0 = 64;

    let mut builder = Builder::new(
        driver,
        usb_cfg,
        cfg_desc,
        bos_desc,
        msos_desc,
        ctrl_desc,
    );

    static CDC_STATE: StaticCell<CdcState<'static>> = StaticCell::new();
    let class = CdcAcmClass::new(&mut builder, CDC_STATE.init(CdcState::new()), 64);

    let (tx, _rx) = class.split();

    let usb = builder.build();

    unwrap!(spawner.spawn(usb_dev_task(usb)));
    unwrap!(spawner.spawn(usb_log_task(tx, pipe)));

    // The following GPIO pins are connected to the wireless chip via SPI.
    // 
    // cf. https://datasheets.raspberrypi.com/picow/pico-2-w-schematic.pdf
    let pin_wl_on = p.PIN_23;
    let pin_wl_d = p.PIN_24;
    let pin_wl_cs = p.PIN_25;
    let pin_wl_clk = p.PIN_29;

    let outpin_wl_cs = Output::new(pin_wl_cs, Level::High);
    let outpin_wl_pwr = Output::new(pin_wl_on, Level::Low);

    let mut pio = Pio::new(p.PIO0, Irqs);
    let spi = PioSpi::new(
        &mut pio.common,
        pio.sm0,
        DEFAULT_CLOCK_DIVIDER,
        pio.irq0,
        outpin_wl_cs,
        pin_wl_d,
        pin_wl_clk,
        p.DMA_CH0,
    );

    let fw = include_bytes!("../cyw43-firmware/43439A0.bin");
    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());

    // Via this `control`, we can operate the wireless chip at last!
    let (_net_device, mut control, runner) = cyw43::new(state, outpin_wl_pwr, spi, fw).await;

    unwrap!(spawner.spawn(cyw43_task(runner)));

    let clm = include_bytes!("../cyw43-firmware/43439A0_clm.bin");
    control.init(clm).await;

    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    let delay = Duration::from_millis(250);
    loop {
        info!("led on!");
        control.gpio_set(0, true).await;
        Timer::after(delay).await;

        info!("led off!");
        control.gpio_set(0, false).await;
        Timer::after(delay).await;
    }
}