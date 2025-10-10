use esp_idf_hal::{delay::FreeRtos, gpio, prelude::*};
use otgi::{obd, wireless};

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
    gatts_server
        .gatts
        .subscribe(move |(gatt_intf, event)| {
            gatts_server.handle_gatts_event(gatt_intf, event);
        })
        .unwrap();

    ble_server.gatts.register_app(wireless::APP_ID).unwrap();

    log::info!("BLE Gap and Gatts initialized");

    let mut high_ref = gpio::PinDriver::output(pins.gpio25).unwrap();
    high_ref.set_high().unwrap();

    let mut driver = obd::ObdDriver::try_new(peripherals.can, pins.gpio33, pins.gpio32, &Default::default()).unwrap();
    driver.start().expect("Failed to start driver");

    let mut data = 0_u16;
    loop {
        log::info!("Run Time: {:?}", driver.query(obd::PID::RunTime));
        FreeRtos::delay_ms(1000);
        log::info!("rpm: {:?}", driver.query(obd::PID::EngineSpeed));
        FreeRtos::delay_ms(1000);
        log::info!("Speed: {:?}", driver.query(obd::PID::VehicleSpeed));
        FreeRtos::delay_ms(1000);
        log::info!("Throttle Position: {:?}", driver.query(obd::PID::ThrottlePosition));
        FreeRtos::delay_ms(1000);

        // TODO: send actual data
        ble_server.indicate(&data.to_le_bytes()).unwrap();
        data += 1;
    }
}

#[cfg(any(esp32s2, not(feature = "experimental")))]
fn main() {
    #[cfg(esp32s2)]
    panic!("ESP32-S2 does not have a BLE radio");

    #[cfg(not(feature = "experimental"))]
    panic!("Use `--features experimental` when build");
}
