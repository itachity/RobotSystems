import os

class Devices():
    HAT_DEVICE_TREE = "/proc/device-tree/"
    HAT_UUIDs = [
        "9daeea78-0000-076e-0032-582369ac3e02", # robothat5 1902v50
        ]

    DEVICES = {
        "robot_hat_v4x": {
            "uuid": None,
            "speaker_enbale_pin": 20,
            "motor_mode": 1,
        }, 
        "robot_hat_v5x": {
            "uuid": HAT_UUIDs[0],
            "speaker_enbale_pin": 12,
            "motor_mode": 2,
        }
    }

    name = ""
    product_id = 0
    product_ver = 0
    uuid = ""
    vendor = ""
    spk_en = 20
    motor_mode = 1

    def __init__(self):
        hat_path = None

        # Desktop/offline fix: /proc/device-tree doesn't exist on Windows
        if os.name == "nt" or not os.path.exists(self.HAT_DEVICE_TREE):
            # keep defaults: spk_en=20, motor_mode=1, etc.
            self.name = "desktop"
            self.vendor = "offline"
            self.uuid = ""
            self.product_id = 0
            self.product_ver = 0
            return

        # Use the constant instead of hardcoding the path
        for file in os.listdir(self.HAT_DEVICE_TREE):
            if 'hat' in file:
                uuid_path = os.path.join(self.HAT_DEVICE_TREE, file, "uuid")
                if os.path.exists(uuid_path) and os.path.isfile(uuid_path):
                    with open(uuid_path, "r") as f:
                        uuid = f.read()[:-1]  # remove \x00
                        if uuid in self.HAT_UUIDs:
                            hat_path = os.path.join(self.HAT_DEVICE_TREE, file)
                            break

        if hat_path is not None:
            with open(os.path.join(hat_path, "product"), "r") as f:
                self.name = f.read()
            with open(os.path.join(hat_path, "product_id"), "r") as f:
                self.product_id = int(f.read()[:-1], 16)
            with open(os.path.join(hat_path, "product_ver"), "r") as f:
                self.product_ver = int(f.read()[:-1], 16)
            with open(os.path.join(hat_path, "uuid"), "r") as f:
                self.uuid = f.read()[:-1]
            with open(os.path.join(hat_path, "vendor"), "r") as f:
                self.vendor = f.read()

            for device in self.DEVICES:
                if self.DEVICES[device]['uuid'] == self.uuid:
                    self.spk_en = self.DEVICES[device]["speaker_enbale_pin"]
                    self.motor_mode = self.DEVICES[device]["motor_mode"]
                    break

if __name__ == "__main__":
    device = Devices()
    print(f'name: {device.name}')
    print(f'product_id: {device.product_id}')
    print(f'product_ver: {device.product_ver}')
    print(f'vendor: {device.vendor}')
    print(f'uuid: {device.uuid}')
    print(f'speaker_enbale_pin: {device.spk_en}')
    print(f'motor_mode: {device.motor_mode}')
