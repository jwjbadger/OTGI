use esp_idf_svc::{
    bt::{
        ble::{
            gap::{AdvConfiguration, BleGapEvent, EspBleGap},
            gatt::{
                server::{EspGatts, GattsEvent},
                AutoResponse, GattCharacteristic, GattDescriptor, GattId, GattInterface,
                GattServiceId, GattStatus, Handle, Permission, Property,
            },
        },
        BdAddr, Ble, BtDriver, BtStatus, BtUuid,
    },
    sys::EspError,
};
use log::info;
use std::sync::{Arc, Condvar, Mutex};

// TODO: Determine proper IDs
pub const APP_ID: u16 = 0;
pub const SERVICE_UUID: u128 = 0x2cbc6002370f577a928681e04f368400;
pub const IND_CHARACTERISTIC_UUID: u128 = 0x56c46fef90390803a71feebcc8650e43;
pub const MAX_CONNECTIONS: usize = 1;

#[derive(Clone)]
pub struct Server {
    pub gap: Arc<EspBleGap<'static, Ble, Arc<BtDriver<'static, Ble>>>>,
    pub gatts: Arc<EspGatts<'static, Ble, Arc<BtDriver<'static, Ble>>>>,
    state: Arc<Mutex<State>>,
    cvar: Arc<Condvar>,
}

#[derive(Debug, Clone)]
struct Connection {
    peer: BdAddr,
    conn_id: Handle,
}

#[derive(Debug, Default)]
struct State {
    connections: Vec<Connection>,
    ind_handle: Option<Handle>,
    gatt_intf: Option<GattInterface>,
    ind_confirmed: Option<BdAddr>,
    service_handle: Option<Handle>,
}

impl Server {
    pub fn new(
        gap: Arc<EspBleGap<'static, Ble, Arc<BtDriver<'static, Ble>>>>,
        gatts: Arc<EspGatts<'static, Ble, Arc<BtDriver<'static, Ble>>>>,
    ) -> Self {
        Self {
            gap,
            gatts,
            state: Default::default(),
            cvar: Arc::new(Condvar::new()),
        }
    }

    pub fn handle_gap_event(&self, event: BleGapEvent) {
        match event {
            BleGapEvent::AdvertisingConfigured(status) => {
                if status != BtStatus::Success {
                    panic!("Failed to start advertising: {:?}", status);
                }

                self.gap.start_advertising().unwrap();
            }
            _ => {
                info!("Received GAP event: {:?}", event)
            }
        }
    }

    pub fn handle_gatts_event(&self, gatt_intf: GattInterface, event: GattsEvent) {
        match event {
            GattsEvent::ServiceRegistered { status, app_id } => {
                info!("GATT Service started with status: {:?}", status);
                if status != GattStatus::Ok {
                    // TODO: propogate errors instead of panicking
                    panic!("Failed to start GATT service: {:?}", status);
                }

                if app_id == APP_ID {
                    self.gap.set_device_name("OTGI").unwrap();
                    self.gap
                        .set_adv_conf(&AdvConfiguration {
                            include_name: true,
                            include_txpower: true,
                            flag: 2,
                            service_uuid: Some(BtUuid::uuid128(SERVICE_UUID)),
                            ..Default::default()
                        })
                        .unwrap();
                    self.gatts
                        .create_service(
                            gatt_intf,
                            &GattServiceId {
                                id: GattId {
                                    inst_id: 0,
                                    uuid: BtUuid::uuid128(SERVICE_UUID),
                                },
                                is_primary: true,
                            },
                            8,
                        )
                        .unwrap();

                    self.state.lock().unwrap().gatt_intf = Some(gatt_intf);
                }
            }
            GattsEvent::ServiceCreated {
                status,
                service_handle,
                ..
            } => {
                if status != GattStatus::Ok {
                    panic!("Failed to create GATT service: {:?}", status);
                }

                self.state.lock().unwrap().service_handle = Some(service_handle);
                self.gatts.start_service(service_handle).unwrap();

                self.gatts
                    .add_characteristic(
                        service_handle,
                        &GattCharacteristic {
                            uuid: BtUuid::uuid128(IND_CHARACTERISTIC_UUID),
                            permissions: (Permission::Write | Permission::Read).into(),
                            properties: (Property::Indicate).into(),
                            max_len: 200,
                            auto_rsp: AutoResponse::ByApp,
                        },
                        &[],
                    )
                    .unwrap();
            }
            GattsEvent::PeerConnected { conn_id, addr, .. } => {
                let mut state = self.state.lock().unwrap();
                if state.connections.len() < MAX_CONNECTIONS {
                    state.connections.push(Connection {
                        peer: addr,
                        conn_id,
                    });

                    self.gap.set_conn_params_conf(addr, 10, 20, 0, 400).unwrap();
                }
            }
            GattsEvent::CharacteristicAdded {
                status,
                attr_handle,
                service_handle,
                char_uuid,
            } => {
                if status != GattStatus::Ok {
                    panic!("Failed to add GATT characteristic: {:?}", status);
                }

                let mut state = self.state.lock().unwrap();

                if state.service_handle == Some(service_handle)
                    && char_uuid == BtUuid::uuid128(IND_CHARACTERISTIC_UUID)
                {
                    state.ind_handle = Some(attr_handle);

                    self.gatts
                        .add_descriptor(
                            service_handle,
                            &GattDescriptor {
                                uuid: BtUuid::uuid16(0x2902), // CCCD
                                permissions: (Permission::Read | Permission::Write).into(),
                            },
                        )
                        .unwrap();
                }
            }
            GattsEvent::PeerDisconnected { addr, .. } => {
                let mut state = self.state.lock().unwrap();
                state.connections.retain(|e| e.peer != addr);
            }
            GattsEvent::Confirm { status, .. } => {
                if status != GattStatus::Ok {
                    panic!("Failed to confirm indication: {:?}", status);
                }

                let mut state = self.state.lock().unwrap();
                if state.ind_confirmed.is_none() {
                    unreachable!("Confirmation sent without indication sent");
                }

                state.ind_confirmed = None;
                self.cvar.notify_all();

                info!("Received confirmation from device");
            }
            _ => {
                info!("Received GATT event: {:?}", event)
            }
        }
    }

    pub fn indicate(&self, data: &[u8]) -> Result<(), EspError> {
        let mut state = self.state.lock().unwrap();

        for i in 0..state.connections.len() {
            if state.ind_confirmed.is_some() {
                state = self.cvar.wait(state).unwrap()
            }

            self.gatts.indicate(
                state.gatt_intf.expect("No GATT interface"),
                state.connections[i].conn_id,
                state.ind_handle.expect("No indicate handle"),
                data,
            )?;

            state.ind_confirmed = Some(state.connections[i].peer);

            info!("Indicated data to {}", state.connections[i].peer);
        }

        Ok(())
    }
}
