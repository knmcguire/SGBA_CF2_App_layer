from cflib.drivers.crazyradio import Crazyradio
import time

cr = Crazyradio(devid=1)

cr.set_channel(56)
cr.set_data_rate(cr.DR_2MPS)

while True:


    # Send multicast packet to P2P port 7
    cr.set_address((0xff,0xe7,0xe7,0xe7,0xe7))
    cr.set_ack_enable(False)
    cr.send_packet( (0xff, 0x80, 0x63, 0x00) )
    print('send')

    time.sleep(0.01)
