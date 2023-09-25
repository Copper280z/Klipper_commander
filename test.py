import serial
import time
import numpy as np

# s = serial.Serial("/dev/cu.usbmodem103", baudrate=460800, timeout=5)
s = serial.Serial("/dev/ttyACM0", baudrate=250000, timeout=5)
time.sleep(1.5)
ack_response = [0x5, 0x12, 0xbd, 0x93, 0x7e]
full_response = [ 0x30, 0x12, 0x0, 0x0, 0x28, 0x78, 0x9c, 0x75, 0x55, 0x5b, 0x4f, 0xdb, 0x4a, 0x10, 0xfe, 0x2b, 0x2b, 0x4b, 0x48, 0xbd, 0x24, 0x34, 0x76, 0x12, 0x12, 0x23, 0xf1, 0x40, 0x51, 0x90, 0xaa, 0x96, 0x96, 0x92, 0x72, 0x5e, 0x8e, 0xaa, 0xd5, 0xc6, 0x3b, 0x71, 0x56, 0xf5, 0x8d, 0x98, 0x83, 0x7e ]
correct_responses = []
for i in range(200):
    print(f"sending message: {i}")
    s.write(bytearray([0x8, 0x11, 0x1, 0x0, 0x28, 0x42, 0x24, 0x7e]))
    # time.sleep(0.005)
    ack = s.read_all()

    # string=""
    # for n,r in zip(ack,ack_response):
    #     string=string+hex(int(n))+f" "
    #     if int(n) != r:
    #         print("****BAD ACK****")
    print(ack)


    body = s.read_all()
    # print(type(ret))
    # print(ret)
    # string=""
    # correct = 1
    # for n,r in zip(body,full_response):
    #     # print(hex(int(n)))
    #     string=string+hex(int(n))+f" "
    #     if int(n) != r:
    #         correct=0
    # correct_responses.append(correct)
    time.sleep(0.002)
    print(body)
print(f'{np.sum(correct_responses)} out of 200')


