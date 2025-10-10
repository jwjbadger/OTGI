#![allow(unexpected_cfgs)]

use esp_idf_hal::{delay::FreeRtos, gpio, prelude::*};
use esp_idf_svc::{
    bt::{
        ble::{gap::EspBleGap, gatt::server::EspGatts},
        Ble, BtDriver,
    },
    nvs::EspDefaultNvsPartition,
};
use otgi::{obd, wireless};
use std::sync::Arc;

#[cfg(all(not(esp32s2), feature = "experimental"))]
fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();

    let nvs = EspDefaultNvsPartition::take().unwrap();
    let bt = Arc::new(BtDriver::<Ble>::new(peripherals.modem, Some(nvs.clone())).unwrap());
    let pins = peripherals.pins;

    let ble_server = wireless::Server::new(
        Arc::new(EspBleGap::new(bt.clone()).unwrap()),
        Arc::new(EspGatts::new(bt.clone()).unwrap()),
    );

    let gap_server = ble_server.clone();
    ble_server
        .gap
        .subscribe(move |event| {
            gap_server.handle_gap_event(event);
        })
        .unwrap();

    let gatts_server = ble_server.clone();
    ble_server
        .gatts
        .subscribe(move |(gatt_intf, event)| {
            gatts_server.handle_gatts_event(gatt_intf, event);
        })
        .unwrap();

    ble_server.gatts.register_app(wireless::APP_ID).unwrap();

    log::info!("BLE Gap and Gatts initialized");

    let mut high_ref = gpio::PinDriver::output(pins.gpio25).unwrap();
    high_ref.set_high().unwrap();

    let mut driver = obd::ObdDriver::try_new(
        peripherals.can,
        pins.gpio33,
        pins.gpio32,
        &Default::default(),
    )
    .unwrap();
    driver.start().expect("Failed to start driver");

    let mut liters_used: f32 = 0.0;
    loop {
        // TODO: See if it's possible to reduce delay
        let maf = driver.query(obd::PID::MassAirFlow);
        FreeRtos::delay_ms(100);
        let stft = driver.query(obd::PID::ShortTermFuelTrimBankOne);
        FreeRtos::delay_ms(100);
        let ltft = driver.query(obd::PID::LongTermFuelTrimBankOne);
        FreeRtos::delay_ms(100);

        if let (Ok(maf), Ok(stft), Ok(ltft)) = (maf, stft, ltft) {
            let usage = (maf * 3600.0) / ((14.7 * (1.0 + ((stft + ltft) / 100.0))) * 740.0);
            liters_used += (usage / 3600.0) * 0.3; // rough approximation (300 ms)
            log::info!(
                "Estimated fuel usage (L/h): {:?}; {:?} liters used in total",
                usage,
                liters_used
            );
        }

        // UNTESTED
        ble_server
            .indicate(&(liters_used as u16).to_le_bytes())
            .unwrap();
    }
}

#[cfg(any(esp32s2, not(feature = "experimental")))]
fn main() {
    #[cfg(esp32s2)]
    panic!("ESP32-S2 does not have a BLE radio");

    #[cfg(not(feature = "experimental"))]
    panic!("Use `--features experimental` when build");
}
