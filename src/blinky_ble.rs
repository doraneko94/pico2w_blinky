#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering};

use embassy_futures::join::join;
use embassy_futures::select::select;
use panic_halt as _;

use static_cell::StaticCell;

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};

use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{PIO0, DMA_CH0};
use embassy_rp::pio::{Pio, InterruptHandler as PioInterruptHandler};

use cyw43_pio::{DEFAULT_CLOCK_DIVIDER, PioSpi};

use trouble_host::gap::{GapConfig, PeripheralConfig};
use trouble_host::gatt::{GattConnection, GattConnectionEvent, GattEvent};
use trouble_host::prelude::{
    appearance, gatt_server, gatt_service, 
    AdStructure, Advertisement, DefaultPacketPool, ExternalController, FromGatt, Host, Peripheral, Runner, Stack,
    BR_EDR_NOT_SUPPORTED, LE_GENERAL_DISCOVERABLE
};
use trouble_host::{self as ble, Address, BleHostError, Controller, Error, HostResources, PacketPool};
use ble::attribute::Uuid;

const DELAY_DEFAULT: u32 = 250;
static DELAY_MS: AtomicU32 = AtomicU32::new(DELAY_DEFAULT);

const UUID_DELAY_SVC_RAW: [u8; 16] = [ // 7e20c8a7-5f47-4a75-9a9e-2d7af5f6b001
    0x01, 0xb0, 0xf6, 0xf5, 0x7a, 0x2d, 0x9e, 0x9a, 0x75, 0x4a, 0x47, 0x5f, 0xa7, 0xc8, 0x20, 0x7e,
];
const UUID_DELAY_CHR_RAW: [u8; 16] = [ // 7e20c8a7-5f47-4a75-9a9e-2d7af5f6b002
    0x02, 0xb0, 0xf6, 0xf5, 0x7a, 0x2d, 0x9e, 0x9a, 0x75, 0x4a, 0x47, 0x5f, 0xa7, 0xc8, 0x20, 0x7e,
];
const UUID_DELAY_SVC: Uuid = Uuid::new_long(UUID_DELAY_SVC_RAW);
const UUID_DELAY_CHR: Uuid = Uuid::new_long(UUID_DELAY_CHR_RAW);

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
});

#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>,
) -> ! {
    runner.run().await
}

const CONNECTIONS_MAX: usize = 1;
const L2CAP_CHANNELS_MAX: usize = 2;

#[gatt_server]
struct Server {
    delay_service: DelayService
}

#[gatt_service(uuid = UUID_DELAY_SVC)]
struct DelayService {
    #[characteristic(uuid = UUID_DELAY_CHR, read, write, notify, value = DELAY_DEFAULT)]
    delay_ms: u32
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

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
    let clm = include_bytes!("../cyw43-firmware/43439A0_clm.bin");
    let btfw = include_bytes!("../cyw43-firmware/43439A0_btfw.bin");

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());

    let (_net_device, bt_device, mut control, runner) = cyw43::new_with_bluetooth(state, outpin_wl_pwr, spi, fw, btfw).await;

    spawner.must_spawn(cyw43_task(runner));
    control.init(clm).await;

    control.gpio_set(0, true).await;
    Timer::after(Duration::from_millis(2000)).await;
    control.gpio_set(0, false).await;

    spawner.must_spawn(led_task(control));

    let controller: ExternalController<_, 10> = ExternalController::new(bt_device);

    run(controller).await;
}

pub async fn run<C>(controller: C)
where
    C: Controller,
{
    let address: Address = Address::random([0xff, 0x8f, 0x1a, 0x05, 0xe4, 0xff]);

    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> = HostResources::new();
    let stack = trouble_host::new(controller, &mut resources).set_random_address(address);
    let Host { mut peripheral, runner, .. } = stack.build();

    let server = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: "Pico2W Delay",
        appearance: &appearance::light_source::GENERIC_LIGHT_SOURCE,
    })).unwrap();

    let _ = join(ble_task(runner), async {
        loop {
            if let Ok(conn) = advertise("Pico2W Delay", &mut peripheral, &server).await {
                let a = gatt_events_task(&server, &conn);
                let b = notify_task(&server, &conn, &stack);
                select(a, b).await;
            }
        }
    })
    .await;
}

#[embassy_executor::task]
async fn led_task(
    mut control: cyw43::Control<'static>,
) -> ! {
    loop {
        let d = DELAY_MS.load(Ordering::Relaxed).max(1) as u64;
        let delay = Duration::from_millis(d);

        control.gpio_set(0, true).await;
        Timer::after(delay).await;

        control.gpio_set(0, false).await;
        Timer::after(delay).await;
    }
}

async fn ble_task<C: Controller, P: PacketPool>(mut runner: Runner<'_, C, P>) {
    loop {
        let _ =  runner.run().await;
    }
}

async fn advertise<'values, 'server, C: Controller>(
    name: &'values str,
    peripheral: &mut Peripheral<'values, C, DefaultPacketPool>,
    server: &'server Server<'values>,
) -> Result<GattConnection<'values, 'server, DefaultPacketPool>, BleHostError<C::Error>> {
    let mut adv = [0; 31];
    let adv_len= AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::ServiceUuids128(&[UUID_DELAY_SVC_RAW]),
        ], 
        &mut adv[..],
    )?;

    let mut scan = [0u8; 31];
    let scan_len = AdStructure::encode_slice(
        &[
            AdStructure::CompleteLocalName(name.as_bytes())
        ], 
        &mut scan[..],
    )?;

    let advertiser = peripheral
        .advertise(
            &Default::default(), 
            Advertisement::ConnectableScannableUndirected { 
                adv_data: &adv[..adv_len], 
                scan_data: &scan[..scan_len], 
            },
        )
        .await?;

    let conn = advertiser.accept().await?.with_attribute_server(server)?;
    Ok(conn)
}

async fn gatt_events_task<P: PacketPool>(
    server: &Server<'_>,
    conn: &GattConnection<'_, '_, P>,
) -> Result<(), Error> {
    let delay = server.delay_service.delay_ms;

    loop {
        match conn.next().await {
            GattConnectionEvent::Disconnected { .. } => break,
            GattConnectionEvent::Gatt { event } => {
                match &event {
                    GattEvent::Read(ev) => {
                        if ev.handle() == delay.handle {
                            let v = server.get(&delay);
                            let _ = v;
                        }
                    }
                    GattEvent::Write(ev) => {
                        if ev.handle() == delay.handle {
                            let data = ev.data();
                            if data.len() == 4 {
                                let v = u32::from_le_bytes([data[0], data[1], data[2], data[3]])
                                    .clamp(1, 10_000);
                                let _ = server.set(&delay, &v);
                                DELAY_MS.store(v, Ordering::Relaxed);
                            }
                        }
                    }
                    _ => {}
                }
                if let Ok(reply) = event.accept() {
                    let _ = reply.send().await;
                }
            }
            _ => {}
        }
    }
    Ok(())
}

async fn notify_task<C: Controller, P: PacketPool>(
    server: &Server<'_>,
    conn: &GattConnection<'_, '_, P>,
    _stack: &Stack<'_, C, P>,
) {
    let ch = server.delay_service.delay_ms;
    loop {
        let v = DELAY_MS.load(Ordering::Relaxed);
        if let Err(_e) = ch.notify(conn, &v).await {
            Timer::after_millis(500).await;
            continue;
        }
        Timer::after_secs(1).await;
    }
}