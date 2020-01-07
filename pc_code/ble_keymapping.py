from bluepy.btle import Peripheral, UUID
from bluepy.btle import Scanner, DefaultDelegate
class ScanDelegate(DefaultDelegate):
    def __init__(self):
        DefaultDelegate.__init__(self)
    def handleDiscovery(self, dev, isNewDev, isNewData):
        if isNewDev:
            print ("Discovered device", dev.addr)
        elif isNewData:
            print ("Received new data from", dev.addr)
scanner = Scanner().withDelegate(ScanDelegate())
devices = list(scanner.scan(10.0))
n=0
for dev in devices:
    if dev.getValueText(9) is None:
        n += 1
        continue
    print ("%d: Device %s (%s), RSSI=%d dB" % (n, dev.addr, dev.addrType, dev.rssi))
    print ("Complete Local Name: %s"%(dev.getValueText(9)))
    
    n += 1
    #for (adtype, desc, value) in dev.getScanData():
    #    print (" %s = %s" % (adtype, desc, value))
number = int(input('Enter your device number: '))
print('Device', number)
print(devices[number].addr)
print ("Connecting...")
dev = Peripheral(devices[number].addr, 'random')
print ("Services...")
for svc in dev.services:
    print (str(svc))
try:
    testService = dev.getServiceByUUID(UUID(0xA000))
    for ch in testService.getCharacteristics():
        print (str(ch))
    ch = dev.getCharacteristics(uuid=UUID(0xA001))[0]
    print (ch)
    if (ch.supportsRead()):
        from pynput.keyboard import Key, Controller
        keyboard = Controller()
        while (True):
            #TODO: read STM32 input and output keymapping
            '''
            from pynput.keyboard import Key, Controller
            keyboard = Controller()
            #Press and release space
            keyboard.press(Key.space)
            keyboard.release(Key.space)
            #Type a lower case A ;this will work even if no key on the physical keyboard  is labelled "A"
            keyboard.press("a")
            keyboard.release("a")
            #Type two upper case As
            keyboard.press("A")
            keyboard.release("A")
            # or 
            with keyboard.pressed(Key.shift):
                keyboard.press("a")
                keyboard.release("a")
            #type "hello world "  using the shortcut type  method
            keyboard.type("hello world")
            '''

            '''
                a[0]: _walk
                a[1]: _direction (0=left, 1=no, 2=right)
                a[2]: _jump
                a[3]: _attack
            '''

            a = ch.read()
            print(a)
            if a[0] == 1:
                if a[1] == 0:
                    keyboard.release(Key.right)
                    keyboard.press(Key.left)
                elif a[1] == 2:
                    keyboard.release(Key.left)
                    keyboard.press(Key.right)
                else:
                    keyboard.release(Key.left)
                    keyboard.release(Key.right)
            else:
                keyboard.release(Key.left)
                keyboard.release(Key.right)

            if a[2] == 1:
                keyboard.press(Key.space)
                keyboard.release(Key.space)

            if a[3] == 1:
                keyboard.press(Key.ctrl)
                keyboard.release(Key.ctrl)


            if a[2] == 1:
                keyboard.press("a")
                keyboard.release("a")
    else:
        print (ch, "does not supports read")
finally:
    dev.disconnect()
