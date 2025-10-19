#![no_std]
#![no_main]

use panic_halt as _;

use cyw43_pio::{DEFAULT_CLOCK_DIVIDER, PioSpi};

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, PIO0, USB};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_rp::usb as rp_usb;
use embassy_time::{Duration, Timer};

use embassy_usb::Builder;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State as CdcState};

use embedded_io_async::Write as _;
use static_cell::StaticCell;

#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"Pico2W USB Delay"),
    embassy_rp::binary_info::rp_program_description!(c"USB CDC prints delay every second"),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

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
async fn usb_print_task(
    mut tx: embassy_usb::class::cdc_acm::Sender<'static, rp_usb::Driver<'static, USB>>,
    delay_ms: u32,
) -> ! {
    tx.wait_connection().await;

    use core::fmt::Write as FmtWrite;
    use heapless::String;

    loop {
        let mut s: String<64> = String::new();
        let _ = core::write!(&mut s, "delay={} ms\r\n", delay_ms);
        let _ = tx.write_all(s.as_bytes()).await;
        Timer::after(Duration::from_secs(1)).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let driver = rp_usb::Driver::new(p.USB, Irqs);

    static USB_CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    static USB_BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    static USB_MSOS_DESCRIPTOR: StaticCell<[u8; 128]> = StaticCell::new();
    static USB_CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();

    let mut config = embassy_usb::Config::new(0x2e8a, 0x000a);
    config.manufacturer = Some("Pico 2 W");
    config.product = Some("USB CDC Example");
    config.serial_number = Some("0001");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    let mut builder = Builder::new(
        driver,
        config,
        USB_CONFIG_DESCRIPTOR.init([0; 256]),
        USB_BOS_DESCRIPTOR.init([0; 256]),
        USB_MSOS_DESCRIPTOR.init([0; 128]),
        USB_CONTROL_BUF.init([0; 64]),
    );

    static CDC_STATE: StaticCell<CdcState<'static>> = StaticCell::new();
    let cdc = CdcAcmClass::new(&mut builder, CDC_STATE.init(CdcState::new()), 64);
    let (tx, _rx) = cdc.split();

    let usb = builder.build();
    spawner.spawn(usb_dev_task(usb)).unwrap();

    // The following GPIO pins are connected to the wireless chip via SPI.
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

    let (_net_device, mut control, runner) = cyw43::new(state, outpin_wl_pwr, spi, fw).await;

    spawner.spawn(cyw43_task(runner)).unwrap();

    let clm = include_bytes!("../cyw43-firmware/43439A0_clm.bin");
    control.init(clm).await;

    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    let delay_ms: u32 = 250;
    spawner.spawn(usb_print_task(tx, delay_ms)).unwrap();

    let delay = Duration::from_millis(delay_ms as u64);
    loop {
        control.gpio_set(0, true).await;
        Timer::after(delay).await;

        control.gpio_set(0, false).await;
        Timer::after(delay).await;
    }
}