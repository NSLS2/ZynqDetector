template <typename NetworkBehavior>
class Network {
public:
    NetworkBehavior behavior;

    void receive() {
        behavior.receiveData();
    }

    void transmit() {
        behavior.transmitData();
    }
};

//========================================

template <typename Detector>
class ZynqDetector {
public:
    Network<Detector> network; // Network now uses the Derived class directly

    void useNetwork() {
        network.receive();
        network.transmit();
    }
};

//========================================


class Germanium : public ZynqDetector<Germanium> {
public:
    void udp_rx() {
        std::cout << "Wifi: Receiving data from DerivedWifi..." << std::endl;
        // Implement Wifi-specific receive logic
    }
    void udp_tx() {
        std::cout << "Wifi: Transmitting data from DerivedWifi..." << std::endl;
        // Implement Wifi-specific transmit logic
    }
};


//========================================

int main() {
    Germanium germanium;
    germanium.useNetwork();

    return 0;
}