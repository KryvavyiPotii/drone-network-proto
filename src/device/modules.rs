use super::*;


fn current_signal_levels_are_not_higher_than_max(
    current_signal_levels: &HashMap<SignalType, SignalLevel>,
    max_signal_levels: &HashMap<SignalType, SignalLevel>
) -> bool {
    current_signal_levels
        .iter()
        .all(|(signal_type, signal_level)|
            *signal_level <= *max_signal_levels
                                .get(signal_type)
                                .unwrap_or(&NO_SIGNAL_LEVEL)
        )
}

fn tx_signal_level_at_by_color(
    tx_signal_level: &SignalLevel, 
    distance_to_rx: Meter
) -> SignalLevel {
    let radius = SignalArea::from(*tx_signal_level).radius();

    if distance_to_rx <= radius * GREEN_SIGNAL_ZONE_COEFFICIENT {
        *tx_signal_level
    } else if distance_to_rx <= radius * YELLOW_SIGNAL_ZONE_COEFFICIENT {
        tx_signal_level.lower_level()
    } else if distance_to_rx <= radius {
        tx_signal_level.lower_level().lower_level()
    } else {
        BLACK_SIGNAL_LEVEL
    }
}

fn tx_signal_level_at_by_strength(
    tx_signal_level: &SignalLevel, 
    distance_to_rx: Meter
) -> SignalLevel {
    tx_signal_level.at(distance_to_rx)
}

fn receive_signal_level_by_color(
    tx_signal_level: SignalLevel, 
    rx_signal_level: SignalLevel,
) -> SignalLevel {
    if tx_signal_level.is_green() {
        rx_signal_level
    } else if rx_signal_level.is_green() {
        tx_signal_level
    } else if tx_signal_level.is_yellow() && rx_signal_level.is_yellow() {
        RED_SIGNAL_LEVEL
    } else {
        BLACK_SIGNAL_LEVEL
    }
}

fn receive_signal_level_by_strength(
    tx_signal_level: SignalLevel, 
    max_rx_signal_level: SignalLevel,
) -> SignalLevel {
    min_signal_level(tx_signal_level, max_rx_signal_level)
}

fn suppress_signal_level_by_color(
    suppressor_signal_level: SignalLevel, 
    rx_signal_level: SignalLevel,
) -> SignalLevel {
    if suppressor_signal_level.is_black() {
        rx_signal_level    
    } else if suppressor_signal_level.is_red() && rx_signal_level.is_green() {
        YELLOW_SIGNAL_LEVEL
    } else if suppressor_signal_level.is_yellow() && rx_signal_level.is_green() 
        || suppressor_signal_level.is_red() && rx_signal_level.is_yellow() {
        RED_SIGNAL_LEVEL
    } else {
        BLACK_SIGNAL_LEVEL
    }
}

fn suppress_signal_level_by_strength(
    suppressor_signal_level: SignalLevel, 
    rx_signal_level: SignalLevel,
) -> SignalLevel {
    rx_signal_level - suppressor_signal_level
}


pub trait Transmitter: Device {
    fn tx_signal_levels(&self) -> &HashMap<SignalType, SignalLevel>;
    fn tx_signal_level(&self, signal_type: &SignalType) -> &SignalLevel;
    fn area(&self, signal_type: &SignalType) -> SignalArea;

    fn connection_distance<P: Position>(
        &self, 
        object: &P, 
        signal_type: &SignalType
    ) -> Option<Meter>;

    fn propagated_signal_level_at<R: Receiver>(
        &self,
        receiver: &R,
        signal_type: &SignalType
    ) -> SignalLevel;
    
    fn propagate_signal_level<R: Receiver>(
        &self,
        receiver: &mut R,
        signal_type: SignalType
    );
}

pub trait Suppressor: Transmitter {
    fn suppress_signal_level<R: Receiver>(
        &self,
        receiver: &mut R,
        signal_type: &SignalType
    );
}

pub trait Receiver: Device {
    fn rx_signal_levels(&self) -> &HashMap<SignalType, SignalLevel>;
    fn rx_signal_level(&self, signal_type: &SignalType) -> &SignalLevel;
    fn receives_signal(&self, signal_type: &SignalType) -> bool;

    fn receive_signal(
        &mut self, 
        signal_type: SignalType,
        signal_level: SignalLevel
    );

    // Return value:
    //     Ok(()) -> message is received.
    //     Err(_) -> message is not received, because...
    // TODO add proper error handling (with custom error enum)
    fn receive_message(
        &mut self,
        signal_type: SignalType,
        message: &Message
    ) -> Result<(), ()>;

    fn signal_level_suppression(
        &mut self,
        suppressor_signal_type: SignalType,
        suppressor_signal_level: SignalLevel
    );
}

pub trait Transceiver: Transmitter + Receiver {}


#[derive(Clone, Copy, Debug)]
pub enum AntennaType {
    Color,
    Strength,
    Dummy
}


#[derive(Clone, Debug)]
pub struct TRXModule {
    antenna: AntennaType,
    max_signal_levels: HashMap<SignalType, SignalLevel>,
    signal_levels: HashMap<SignalType, SignalLevel>
}

impl TRXModule {
    pub fn build(
        antenna: AntennaType,
        max_signal_levels: HashMap<SignalType, SignalLevel>,
        signal_levels: HashMap<SignalType, SignalLevel>
    ) -> Result<Self, &'static str> {
        if !current_signal_levels_are_not_higher_than_max(
            &signal_levels, 
            &max_signal_levels
        ) {
            return Err("Signal levels are higher than max signal levels");
        }

        Ok(
            Self {
                antenna,
                max_signal_levels,
                signal_levels
            }
        )
    }

    pub fn antenna(&self) -> &AntennaType {
        &self.antenna
    }

    pub fn max_signal_levels(&self) -> &HashMap<SignalType, SignalLevel> {
        &self.max_signal_levels
    }
    
    pub fn max_signal_level(&self, signal_type: &SignalType) -> &SignalLevel {
        self.max_signal_levels
            .get(signal_type)
            .unwrap_or(&NO_SIGNAL_LEVEL)
    }
    
    pub fn signal_levels(&self) -> &HashMap<SignalType, SignalLevel> {
        &self.signal_levels
    }

    pub fn signal_level(&self, signal_type: &SignalType) -> &SignalLevel {
        self.signal_levels
            .get(signal_type)
            .unwrap_or(&NO_SIGNAL_LEVEL)
    }
    
    pub fn set_signal_levels(
        &mut self, 
        signal_levels: HashMap<SignalType, SignalLevel>
    ) {
        signal_levels
            .iter()
            .for_each(|(signal_type, signal_level)|
                self.set_signal_level(*signal_type, *signal_level)
            );
    }
    
    pub fn set_signal_level(
        &mut self, 
        signal_type: SignalType,
        signal_level: SignalLevel
    ) {
        let max_signal_level = *self.max_signal_level(&signal_type);

        let new_signal_level = min_signal_level(
            signal_level,
            max_signal_level
        );

        self.signal_levels.insert(signal_type, new_signal_level);
    }
}

// By default we create an empty TRXModule that can not function.
impl Default for TRXModule {
    fn default() -> Self {
        Self {
            antenna: AntennaType::Dummy,
            max_signal_levels: HashMap::new(),
            signal_levels: HashMap::new()
        }
    }
}


#[derive(Clone, Debug, Default)]
pub struct TRXSystem {
    tx_module: TRXModule,
    rx_module: TRXModule,
}

impl TRXSystem {
    pub fn new(
        tx_module: TRXModule,
        rx_module: TRXModule,
    ) -> Self {
        Self { tx_module, rx_module }
    }

    pub fn tx_module(&self) -> &TRXModule {
        &self.tx_module
    }
    
    pub fn rx_module(&self) -> &TRXModule {
        &self.rx_module
    }
    
    pub fn tx_module_mut(&mut self) -> &mut TRXModule {
        &mut self.tx_module
    }
    
    pub fn rx_module_mut(&mut self) -> &mut TRXModule {
        &mut self.rx_module
    }

    pub fn max_tx_signal_level(
        &self, 
        signal_type: &SignalType
    ) -> &SignalLevel {
        self.tx_module
            .max_signal_levels()
            .get(signal_type)
            .unwrap_or(&NO_SIGNAL_LEVEL)
    }
    
    pub fn tx_signal_levels(&self) -> &HashMap<SignalType, SignalLevel> {
        &self.tx_module.signal_levels()
    }

    pub fn tx_signal_level(&self, signal_type: &SignalType) -> &SignalLevel {
        self.tx_module.signal_level(signal_type)
    }

    pub fn area(&self, signal_type: &SignalType) -> SignalArea {
        SignalArea::from(self.tx_signal_level(signal_type))
    }
    
    pub fn set_tx_signal_levels(
        &mut self, 
        signal_levels: HashMap<SignalType, SignalLevel>
    ) {
        self.tx_module.set_signal_levels(signal_levels);
    }
    
    pub fn set_tx_signal_level(
        &mut self, 
        signal_type: SignalType,
        signal_level: SignalLevel
    ) {
        self.rx_module.set_signal_level(signal_type, signal_level);
    }
    
    pub fn max_rx_signal_level(
        &self, 
        signal_type: &SignalType
    ) -> &SignalLevel {
        self.rx_module
            .max_signal_levels()
            .get(signal_type)
            .unwrap_or(&NO_SIGNAL_LEVEL)
    }
    
    pub fn rx_signal_levels(&self) -> &HashMap<SignalType, SignalLevel> {
        &self.rx_module.signal_levels
    }
    
    pub fn rx_signal_level(&self, signal_type: &SignalType) -> &SignalLevel {
        self.rx_module.signal_level(signal_type)
    }
    
    pub fn set_rx_signal_levels(
        &mut self, 
        signal_levels: HashMap<SignalType, SignalLevel>
    ) {
        self.rx_module.set_signal_levels(signal_levels);
    }
    
    pub fn set_rx_signal_level(
        &mut self, 
        signal_type: SignalType,
        signal_level: SignalLevel
    ) {
        self.rx_module.set_signal_level(signal_type, signal_level);
    }

    pub fn connection_distance(
        &self, 
        distance: Meter, 
        signal_type: &SignalType
    ) -> Option<Meter> {
        let tx_signal_level = self.tx_signal_level(signal_type);

        // Optimization (if-statement is cheaper than calculating a formula with
        // sqrt).
        if tx_signal_level.is_black() {
            return None;
        }

        if !self
            .tx_signal_level_at(distance, signal_type)
            .is_black()
        {
            Some(distance)
        } else {
            None
        }
    }
    
    pub fn tx_signal_level_at(
        &self, 
        distance: Meter, 
        signal_type: &SignalType
    ) -> SignalLevel {
        match self.tx_module.antenna() {
            AntennaType::Color => tx_signal_level_at_by_color(
                self.tx_signal_level(signal_type),
                distance
            ),
            AntennaType::Strength => tx_signal_level_at_by_strength(
                self.tx_signal_level(signal_type),
                distance
            ),
            AntennaType::Dummy => NO_SIGNAL_LEVEL
        }
    }

    pub fn suppress_signal_level(
        &mut self,
        suppressor_signal_type: SignalType,
        suppressor_signal_level: SignalLevel
    ) {
        let suppressed_signal_level = match self.rx_module.antenna() {
            AntennaType::Color => suppress_signal_level_by_color(
                suppressor_signal_level, 
                *self.rx_signal_level(&suppressor_signal_type)
            ),
            AntennaType::Strength => suppress_signal_level_by_strength(
                suppressor_signal_level, 
                *self.rx_signal_level(&suppressor_signal_type)
            ), 
            AntennaType::Dummy => NO_SIGNAL_LEVEL 
        };

        self.set_rx_signal_level(
            suppressor_signal_type, 
            suppressed_signal_level
        );
    }

    pub fn receives_signal_level(&self, signal_type: &SignalType) -> bool {
        *self.rx_signal_level(signal_type) > BLACK_SIGNAL_LEVEL
    }

    pub fn receive_signal_level(
        &mut self,
        tx_signal_type: SignalType,
        tx_signal_level: SignalLevel
    ) {
        let received_signal_level = match self.rx_module.antenna() {
            AntennaType::Color => receive_signal_level_by_color(
                tx_signal_level, 
                *self.rx_signal_level(&tx_signal_type)
            ),
            AntennaType::Strength => receive_signal_level_by_strength(
                tx_signal_level, 
                *self.max_rx_signal_level(&tx_signal_type)
            ), 
            AntennaType::Dummy => NO_SIGNAL_LEVEL
        };

        self.set_rx_signal_level(tx_signal_type, received_signal_level);
    }

    pub fn receive_message(
        &mut self,
        signal_type: SignalType,
        message: &Message
    ) -> Result<(), ()> {
        match self.rx_module.antenna() {
            AntennaType::Color => Ok(()),
            AntennaType::Strength => {
                let current_signal_level = self.rx_signal_level(&signal_type);
                let new_signal_level = *current_signal_level - message.cost();
                
                if new_signal_level < NO_SIGNAL_LEVEL {
                    return Err(());
                }

                self.set_rx_signal_level(signal_type, new_signal_level);

                Ok(())
            },
            AntennaType::Dummy => Err(())
        }
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    const VERY_BIG_NUMBER: usize = 1_000;

    #[test]
    fn green_tx_signal_level_at_rx_by_color() {
        let tx_signal_level = GREEN_SIGNAL_LEVEL;
        let radius = SignalArea::from(&tx_signal_level).radius();

        assert!(
            tx_signal_level_at_by_color(
                &tx_signal_level, 
                radius * GREEN_SIGNAL_ZONE_COEFFICIENT
            ).is_green()
        );
        assert!(
            tx_signal_level_at_by_color(
                &tx_signal_level, 
                radius * YELLOW_SIGNAL_ZONE_COEFFICIENT
            ).is_yellow()
        );
        assert!(
            tx_signal_level_at_by_color(
                &tx_signal_level, 
                radius * RED_SIGNAL_ZONE_COEFFICIENT
            ).is_red()
        );
        assert!(
            tx_signal_level_at_by_color(
                &tx_signal_level, 
                radius + 1.0
            ).is_black()
        );
    }
    
    #[test]
    fn yellow_tx_signal_level_at_rx_by_color() {
        let tx_signal_level = YELLOW_SIGNAL_LEVEL;
        let radius = SignalArea::from(&tx_signal_level).radius();

        assert!(
            tx_signal_level_at_by_color(
                &tx_signal_level, 
                radius * GREEN_SIGNAL_ZONE_COEFFICIENT
            ).is_yellow()
        );
        assert!(
            tx_signal_level_at_by_color(
                &tx_signal_level, 
                radius * YELLOW_SIGNAL_ZONE_COEFFICIENT
            ).is_red()
        );
        assert!(
            tx_signal_level_at_by_color(
                &tx_signal_level, 
                radius * RED_SIGNAL_ZONE_COEFFICIENT
            ).is_black()
        );
        assert!(
            tx_signal_level_at_by_color(
                &tx_signal_level, 
                radius + 1.0
            ).is_black()
        );
    }   
        
    #[test]
    fn red_tx_signal_level_at_rx_by_color() {
        let tx_signal_level = RED_SIGNAL_LEVEL;
        let radius = SignalArea::from(&tx_signal_level).radius();

        assert!(
            tx_signal_level_at_by_color(
                &tx_signal_level, 
                radius * GREEN_SIGNAL_ZONE_COEFFICIENT
            ).is_red()
        );
        assert!(
            tx_signal_level_at_by_color(
                &tx_signal_level, 
                radius * YELLOW_SIGNAL_ZONE_COEFFICIENT
            ).is_black()
        );
        assert!(
            tx_signal_level_at_by_color(
                &tx_signal_level, 
                radius * RED_SIGNAL_ZONE_COEFFICIENT
            ).is_black()
        );
        assert!(
            tx_signal_level_at_by_color(
                &tx_signal_level, 
                radius + 1.0
            ).is_black()
        );
    }

    #[test]
    fn black_tx_signal_level_at_rx_by_color() {
        let tx_signal_level = BLACK_SIGNAL_LEVEL;
        let radius = SignalArea::from(&tx_signal_level).radius();

        assert!(
            tx_signal_level_at_by_color(
                &tx_signal_level, 
                radius * GREEN_SIGNAL_ZONE_COEFFICIENT
            ).is_black()
        );
        assert!(
            tx_signal_level_at_by_color(
                &tx_signal_level, 
                radius * YELLOW_SIGNAL_ZONE_COEFFICIENT
            ).is_black()
        );
        assert!(
            tx_signal_level_at_by_color(
                &tx_signal_level, 
                radius * RED_SIGNAL_ZONE_COEFFICIENT
            ).is_black()
        );
        assert!(
            tx_signal_level_at_by_color(
                &tx_signal_level, 
                radius + 1.0
            ).is_black()
        );
    }

    #[test]
    fn green_tx_signal_level_at_rx_by_strength() {
        let tx_signal_level = GREEN_SIGNAL_LEVEL;
        let radius = SignalArea::from(&tx_signal_level).radius();
        
        assert!(
            tx_signal_level_at_by_strength(
                &tx_signal_level, 
                radius / 2.0
            ) <= tx_signal_level
        );
        assert!(
            tx_signal_level_at_by_strength(
                &tx_signal_level, 
                radius + 1.0
            ) <= BLACK_SIGNAL_LEVEL
        );
    }

    #[test]
    fn yellow_tx_signal_level_at_rx_by_strength() {
        let tx_signal_level = YELLOW_SIGNAL_LEVEL;
        let radius = SignalArea::from(&tx_signal_level).radius();

        assert!(
            tx_signal_level_at_by_strength(
                &tx_signal_level, 
                radius / 2.0
            ) <= tx_signal_level
        );
        assert!(
            tx_signal_level_at_by_strength(
                &tx_signal_level, 
                radius + 1.0
            ) <= BLACK_SIGNAL_LEVEL
        );
    }   
        
    #[test]
    fn red_tx_signal_level_at_rx_by_strength() {
        let tx_signal_level = RED_SIGNAL_LEVEL;
        let radius = SignalArea::from(&tx_signal_level).radius();

        assert!(
            tx_signal_level_at_by_strength(
                &tx_signal_level, 
                radius / 2.0
            ) <= tx_signal_level
        );
        assert!(
            tx_signal_level_at_by_strength(
                &tx_signal_level, 
                radius + 1.0
            ) <= BLACK_SIGNAL_LEVEL
        );
    }

    #[test]
    fn black_tx_signal_level_at_rx_by_strength() {
        let tx_signal_level = BLACK_SIGNAL_LEVEL;
        let radius = SignalArea::from(&tx_signal_level).radius();

        assert!(
            tx_signal_level_at_by_strength(
                &tx_signal_level, 
                radius / 2.0
            ) <= tx_signal_level 
        );
        assert!(
            tx_signal_level_at_by_strength(
                &tx_signal_level, 
                radius + 1.0
            ) <= BLACK_SIGNAL_LEVEL
        );
    }

    #[test]
    fn receive_green_signal_level_with_green_max_by_color() {
        assert!(
            receive_signal_level_by_color(
                GREEN_SIGNAL_LEVEL, 
                GREEN_SIGNAL_LEVEL
            ).is_green()
        );
    }

    #[test]
    fn receive_higher_signal_level_with_lower_max_by_color() {
        assert!(
            receive_signal_level_by_color(
                GREEN_SIGNAL_LEVEL, 
                YELLOW_SIGNAL_LEVEL
            ).is_yellow()
        );
    }

    #[test]
    fn receive_lower_signal_level_with_green_max_by_color() {
        assert!(
            receive_signal_level_by_color(
                RED_SIGNAL_LEVEL, 
                GREEN_SIGNAL_LEVEL
            ).is_red()
        );
    }

    #[test]
    fn receive_yellow_signal_level_with_yellow_max_by_color() {
        assert!(
            receive_signal_level_by_color(
                YELLOW_SIGNAL_LEVEL, 
                YELLOW_SIGNAL_LEVEL
            ).is_red()
        );
    }

    #[test]
    fn receive_lower_signal_level_with_yellow_max_by_color() {
        assert!(
            receive_signal_level_by_color(
                RED_SIGNAL_LEVEL, 
                YELLOW_SIGNAL_LEVEL
            ).is_black()
        );
    }

    #[test]
    fn receive_signal_level_with_even_max_by_strength() {
        let tx_signal_level = SignalLevel::from(5.0);
        let max_rx_signal_level = SignalLevel::from(5.0);

        assert_eq!(
            max_rx_signal_level,
            receive_signal_level_by_strength(
                tx_signal_level, 
                max_rx_signal_level
            )
        );
    }
   
    #[test]
    fn receive_higher_signal_level_with_lower_max_by_strength() {
        let tx_signal_level = SignalLevel::from(15.0);
        let max_rx_signal_level = SignalLevel::from(5.0);

        assert_eq!(
            max_rx_signal_level,
            receive_signal_level_by_strength(
                tx_signal_level, 
                max_rx_signal_level
            )
        );
    }
    
    #[test]
    fn receive_lower_signal_level_with_higher_max_by_strength() {
        let tx_signal_level = SignalLevel::from(5.0);
        let max_rx_signal_level = SignalLevel::from(15.0);

        assert_eq!(
            tx_signal_level,
            receive_signal_level_by_strength(
                tx_signal_level, 
                max_rx_signal_level
            )
        );
    }

    #[test]
    fn not_suppress_signal_level_by_color() {
        let rx_signal_level = YELLOW_SIGNAL_LEVEL;
        
        assert_eq!(
            YELLOW_SIGNAL_LEVEL,
            suppress_signal_level_by_color(
                BLACK_SIGNAL_LEVEL, 
                rx_signal_level
            )
        );
    }
    
    #[test]
    fn partially_suppress_green_signal_level_with_red_by_color() {
        let suppressor_signal_level = RED_SIGNAL_LEVEL;
        let rx_signal_level = GREEN_SIGNAL_LEVEL;
        
        assert!(
            suppress_signal_level_by_color(
                suppressor_signal_level, 
                rx_signal_level
            ).is_yellow()
        );
    }
    
    #[test]
    fn partially_suppress_green_signal_level_with_yellow_by_color() {
        let suppressor_signal_level = YELLOW_SIGNAL_LEVEL;
        let rx_signal_level = GREEN_SIGNAL_LEVEL;
        
        assert!(
            suppress_signal_level_by_color(
                suppressor_signal_level, 
                rx_signal_level
            ).is_red()
        );
    }
    
    #[test]
    fn partially_suppress_yellow_signal_level_with_red_by_color() {
        let suppressor_signal_level = YELLOW_SIGNAL_LEVEL;
        let rx_signal_level = GREEN_SIGNAL_LEVEL;
        
        assert!(
            suppress_signal_level_by_color(
                suppressor_signal_level, 
                rx_signal_level
            ).is_red()
        );
    }
    
    #[test]
    fn completely_suppress_signal_level_by_color() {
        let suppressor_signal_level = GREEN_SIGNAL_LEVEL;
        let rx_signal_level = GREEN_SIGNAL_LEVEL;
        
        assert!(
            suppress_signal_level_by_color(
                suppressor_signal_level, 
                rx_signal_level
            ).is_black()
        );
    }
    
    #[test]
    fn suppress_even_signal_level_by_strength() {
        let suppressor_signal_level = SignalLevel::from(5.0);
        let rx_signal_level = SignalLevel::from(5.0);
        
        assert!(
            suppress_signal_level_by_strength(
                suppressor_signal_level, 
                rx_signal_level
            ).is_black()
        );
    }
    
    #[test]
    fn suppress_lower_signal_level_by_strength() {
        let suppressor_signal_level = SignalLevel::from(15.0);
        let rx_signal_level = SignalLevel::from(5.0);
        
        assert!(
            suppress_signal_level_by_strength(
                suppressor_signal_level, 
                rx_signal_level
            ).is_black()
        );
    }
    
    #[test]
    fn suppress_higher_signal_level_by_strength() {
        let suppressor_signal_level = SignalLevel::from(5.0);
        let rx_signal_level = SignalLevel::from(15.0);
        
        assert!(
            !suppress_signal_level_by_strength(
                suppressor_signal_level, 
                rx_signal_level
            ).is_black()
        );
    }

    #[test]
    fn setting_even_or_too_high_rx_signal_levels() {
        let max_rx_signal_levels = HashMap::from([
            (SignalType::Control, YELLOW_SIGNAL_LEVEL),
            (SignalType::GPS, YELLOW_SIGNAL_LEVEL)
        ]);
        let rx_signal_levels = HashMap::from([
            (SignalType::Control, YELLOW_SIGNAL_LEVEL),
            (SignalType::GPS, RED_SIGNAL_LEVEL)
        ]);

        let mut rx_system = TRXSystem::new(
            TRXModule::default(),
            TRXModule::build(
                AntennaType::Strength,
                max_rx_signal_levels,
                rx_signal_levels
            ).unwrap()
        );

        rx_system.set_rx_signal_level(SignalType::Control, GREEN_SIGNAL_LEVEL);
        rx_system.set_rx_signal_level(SignalType::GPS, YELLOW_SIGNAL_LEVEL);

        assert!(
            rx_system
                .rx_signal_level(&SignalType::Control)
                .is_yellow()
        );
        assert!(
            rx_system
                .rx_signal_level(&SignalType::GPS)
                .is_yellow()
        );
    }

    #[test]
    fn set_signal_level_higher_than_max_for_trx_module() {
        let absent_max_signal_levels = HashMap::new();
        let current_signal_levels = HashMap::from([
            (SignalType::GPS, YELLOW_SIGNAL_LEVEL)
        ]);

        assert!(
            !current_signal_levels_are_not_higher_than_max(
                &current_signal_levels, 
                &absent_max_signal_levels
            )
        );
        
        let too_low_max_signal_levels = HashMap::from([
            (SignalType::GPS, RED_SIGNAL_LEVEL)
        ]);

        assert!(
            !current_signal_levels_are_not_higher_than_max(
                &current_signal_levels, 
                &too_low_max_signal_levels
            )
        );

        let correct_max_signal_levels = HashMap::from([
            (SignalType::GPS, YELLOW_SIGNAL_LEVEL)
        ]);

        assert!(
            current_signal_levels_are_not_higher_than_max(
                &current_signal_levels, 
                &correct_max_signal_levels
            )
        );
    }

    #[test]
    fn is_receiving_signal() {
        let dummy_trx_system = TRXSystem::default();

        assert!(!dummy_trx_system.receives_signal_level(&SignalType::GPS));
    
        let max_rx_signal_levels = HashMap::from([
            (SignalType::GPS, GREEN_SIGNAL_LEVEL)
        ]);
        let rx_signal_levels = HashMap::from([
            (SignalType::GPS, YELLOW_SIGNAL_LEVEL)
        ]);

        let rx_system = TRXSystem::new(
            TRXModule::default(),
            TRXModule::build(
                AntennaType::Strength,
                max_rx_signal_levels,
                rx_signal_levels
            ).unwrap()
        );

        assert!(rx_system.receives_signal_level(&SignalType::GPS));
    }

    #[test]
    fn receive_message_on_strength_antenna_with_strength_decrease() {
        let signal_type = SignalType::GPS;
        let message = Message::new(0, MessageType::ChangeGoal(Goal::Attack));

        let barely_green_signal_level = 
            YELLOW_SIGNAL_LEVEL + SignalStrength::new(0.1);

        let mut strength_rx_system = TRXSystem::new(
            TRXModule::default(),
            TRXModule::build(
                AntennaType::Strength,
                HashMap::from([(signal_type, GREEN_SIGNAL_LEVEL)]),
                HashMap::from([(signal_type, barely_green_signal_level)])
            ).unwrap()
        );

        assert!(
            strength_rx_system
                .receive_message(signal_type, &message)
                .is_ok()
        );
        assert_eq!(
            *strength_rx_system.rx_signal_level(&signal_type),
            barely_green_signal_level - message.cost()
        );

        let mut failed_message_receive = false;
        for _ in 0..VERY_BIG_NUMBER {
            if strength_rx_system
                .receive_message(signal_type, &message)
                .is_err() 
            {
                failed_message_receive = true;
                break;
            }
        }

        assert!(failed_message_receive);
        assert!(
            *strength_rx_system
                .rx_signal_level(&signal_type) <= BLACK_SIGNAL_LEVEL
        );
    }

    #[test]
    fn receive_message_on_color_antenna() {
        let signal_type = SignalType::GPS;
        let message = Message::new(0, MessageType::ChangeGoal(Goal::Attack));
        
        let barely_green_signal_level = 
            YELLOW_SIGNAL_LEVEL + SignalStrength::new(1.0);
        
        let mut color_rx_system = TRXSystem::new(
            TRXModule::default(),
            TRXModule::build(
                AntennaType::Color,
                HashMap::from([(signal_type, GREEN_SIGNAL_LEVEL)]),
                HashMap::from([(signal_type, barely_green_signal_level)])
            ).unwrap()
        );

        assert!(
            color_rx_system
                .receive_message(signal_type, &message)
                .is_ok()
        );
        assert_eq!(
            *color_rx_system.rx_signal_level(&signal_type),
            barely_green_signal_level 
        );
    }

    #[test]
    fn not_receive_message_on_dummy_antenna() {
        let signal_type = SignalType::GPS;
        let message = Message::new(0, MessageType::ChangeGoal(Goal::Attack));
        
        let mut dummy_rx_system = TRXSystem::new(
            TRXModule::default(),
            TRXModule::build(
                AntennaType::Dummy,
                HashMap::new(),
                HashMap::new(),
            ).unwrap()
        );

        assert!(
            dummy_rx_system
                .receive_message(signal_type, &message)
                .is_err()
        );
    }
}
