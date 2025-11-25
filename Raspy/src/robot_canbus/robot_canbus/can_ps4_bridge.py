import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import can


# ================================================
# CONFIGuración del bus CAN
# ================================================
bus = can.interface.Bus(channel='can0', bustype='socketcan')


def send_pwm(m1, m2, m3, m4):
    """Enviar 4 PWM por CAN (-100 a 100)"""

    m1 = max(-100, min(100, int(m1)))
    m2 = max(-100, min(100, int(m2)))
    m3 = max(-100, min(100, int(m3)))
    m4 = max(-100, min(100, int(m4)))

    msg12 = can.Message(
        arbitration_id=0x120,
        data=[m1 & 0xFF, m2 & 0xFF],
        is_extended_id=False
    )

    msg34 = can.Message(
        arbitration_id=0x121,
        data=[m3 & 0xFF, m4 & 0xFF],
        is_extended_id=False
    )

    bus.send(msg12)
    bus.send(msg34)

    print(f"[CAN] PWM -> M1:{m1}  M2:{m2}  M3:{m3}  M4:{m4}")


class CANPS4Bridge(Node):
    def __init__(self):
        super().__init__('can_ps4_bridge')

        # Recibe TODOS los datos del mando PS4
        self.create_subscription(Float32MultiArray, '/Data_PS4', self.ps4_callback, 10)

        self.get_logger().info("Nodo CAN-PS4 iniciado ✔ (con lógica de movimiento)")

    def ps4_callback(self, msg):
        data = msg.data

        if len(data) < 20:
            self.get_logger().warn("Mensaje PS4 incompleto.")
            return

        # ================================
        # Mapeo del array
        # ================================
        LX = data[0]
        LY = data[1]
        RX = data[2]
        RY = data[3]

        L2 = data[4]
        R2 = data[5]

        # Botones 
        Square  = data[6]
        Cross   = data[7]
        Circle  = data[8]
        Triangle= data[9]
        L1      = data[10]
        R1      = data[11]
        Share   = data[12]
        Options = data[13]
        L3      = data[14]
        R3      = data[15]
        PS      = data[16]

        UP      = data[17]
        DOWN    = data[18]
        LEFT    = data[19]
        RIGHT   = data[20] if len(data) > 20 else 0

        # ================================
        # LOGICA DE MOVIMIENTO
        # ================================
        
        # LY controla velocidad
        # LX controla giro
        # R2 puede ser turbo 
        # L2 puede ser freno 

        left_pwm  = (LY * 100) + (LX * 50)
        right_pwm = (LY * 100) - (LX * 50)

        # Limitar
        left_pwm  = max(-100, min(100, left_pwm))
        right_pwm = max(-100, min(100, right_pwm))

        # ================================
        # Enviar a los 4 motores
        # ================================
        send_pwm(left_pwm, right_pwm, left_pwm, right_pwm)

        # Debug
        print(f"[PS4] LX:{LX:.2f} LY:{LY:.2f}  -> L:{left_pwm:.0f} R:{right_pwm:.0f}")


def main(args=None):
    rclpy.init(args=args)
    node = CANPS4Bridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
