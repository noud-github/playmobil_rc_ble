from machine import Pin, Timer
from time import sleep
import network
import bluetooth
#from umqtt.simple import MQTTClient
from micropython import const
from ble_advertising import advertising_payload
from servo import Servo

led = Pin('LED', Pin.OUT)

sg90_servo = Servo(pin=12) 

timer = Timer()

def blink(timer):
    led.toggle()

timer.init(period=1000, mode=Timer.PERIODIC, callback=blink)

_IRQ_CENTRAL_CONNECT = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_IRQ_GATTS_WRITE = const(3)

_FLAG_WRITE = const(0x0008)
_FLAG_NOTIFY = const(0x0010)
_FLAG_WRITE_NORESPONSE = const(0x0004)

_PMRC_UUID = bluetooth.UUID("bc2f4cc6-aaef-4351-9034-d66268e328f0")
_PMRC_RX = (
    bluetooth.UUID("06d1e5e7-79ad-4a71-8faa-373789f7d93c"),
    _FLAG_NOTIFY | _FLAG_WRITE | _FLAG_WRITE_NORESPONSE ,
)
_PMRC_SERVICE = (
    _PMRC_UUID,
    (_PMRC_RX,),
)

MAX_ANGELS = {
        1: 45,
        2: 55,
        3: 70,
        4: 85,
        5: 100
    }

# org.bluetooth.characteristic.gap.appearance.xml
_ADV_APPEARANCE_GENERIC_COMPUTER = const(128)


class BLEPMRC:
    def __init__(self, ble, name="PM-RC RP2040", rxbuf=100):
        self._ble = ble
        self._ble.active(True)
        self._ble.irq(self._irq)
        ((self._rx_handle,),) = self._ble.gatts_register_services((_PMRC_SERVICE,))
        # Increase the size of the rx buffer and enable append mode.
        self._ble.gatts_set_buffer(self._rx_handle, rxbuf, True)
        self._connections = set()
        self._rx_buffer = bytearray()
        self._handler = None
        self._handler_connect = None
        # Optionally add services=[_PMRC_UUID], but this is likely to make the payload too large.
        self._payload = advertising_payload(name=name, appearance=_ADV_APPEARANCE_GENERIC_COMPUTER)
        self._advertise()

    def irq(self, handler):
        self._handler = handler

    def connect(self, handler):
        self._handler_connect = handler

    def _connect(self, state):
        if self._handler_connect:
            self._handler_connect(state)

    def _irq(self, event, data):
        # Track connections so we can send notifications.
        if event == _IRQ_CENTRAL_CONNECT:
            # only one connection allowed!!
            if len(self._connections) == 0:
                conn_handle, _, _ = data
                self._connections.add(conn_handle)
                timer.deinit()
                led.value(1)
                self._connect(True)
        elif event == _IRQ_CENTRAL_DISCONNECT:
            self._connect(False)
            led.value(1)
            timer.init(period=1000, mode=Timer.PERIODIC, callback=blink)
            conn_handle, _, _ = data
            if conn_handle in self._connections:
                self._connections.remove(conn_handle)
            # Start advertising again to allow a new connection.
            self._advertise()
            
            
        elif event == _IRQ_GATTS_WRITE:
            conn_handle, value_handle = data
            if conn_handle in self._connections and value_handle == self._rx_handle:
                self._rx_buffer += self._ble.gatts_read(self._rx_handle)
                if self._handler:
                    self._handler()

    def any(self):
        return len(self._rx_buffer)

    def read(self, sz=None):
        if not sz:
            sz = len(self._rx_buffer)
        result = self._rx_buffer[0:sz]
        self._rx_buffer = self._rx_buffer[sz:]
        return result

    def notify(self, data):
        for conn_handle in self._connections:
            self._ble.gatts_notify(conn_handle, self._rx_handle, data)

    def close(self):
        for conn_handle in self._connections:
            self._ble.gap_disconnect(conn_handle)
        self._connections.clear()

    def _advertise(self, interval_us=500000):
        self._ble.gap_advertise(interval_us, adv_data=self._payload)


class LIGHT():
    def __init__(self, on = False):
        self._on = on
        self._handler = None
    
    def __bool__(self):
        return self._on

    def __str__(self):
        if self._on:
            return 'on'
        else:
            return 'off'

    def irq(self, handler):
        self._handler = handler

    def _irq(self):
        if self._handler:
            self._handler()

    def on(self):
        old = self._on
        self._on = True
        if not old:
            self._irq()
    
    def off(self):
        old = self._on
        self._on = False
        if old:
            self._irq()

    def set(self, value):
        if value == 0x02 :
            self.on()
        else:
            self.off()

    def state(self):
        return self._on


class SERVO():
    def __init__(self, angle = None, inMin = 0, inMax = 0xFF, min_angle = 0, max_angle = 180 ):
        self._inMin = inMin
        self._inMax = inMax
        self._min_angle = min_angle
        self._max_angle = max_angle
        self._center = int((self._min_angle + self._max_angle) /2)
        self._handler = None
        #set default angle to center 
        if angle is None:
           self.center() 

    def __int__(self):
        return int(self._angle)

    def __call__(self,value):
        old = self._angle
        self._angle = self._num_to_range(value,self._inMin,self._inMax,self._min_angle,self._max_angle)
        if old != self._angle:
            self._irq()
    
    def irq(self, handler):
        self._handler = handler

    def _irq(self):
        if self._handler:
            self._handler()
    
    def _num_to_range(self, num, inMin, inMax, outMin, outMax):
        return round(outMin + (float(num - inMin) / float(inMax - inMin) * (outMax - outMin)))

    def center(self):
        self._angle = self._center 


class DRIVE(SERVO):
    def __init__(self, angle = None, factor = 3, inMin = 0, inMax = 0xFF, min_angle = 0, max_angle = 180 ):
        #self._drive_min_angle = min_angle
        self._drive_max_angle = max_angle
        SERVO.__init__(self, angle=angle, inMin=inMin , inMax=inMax , min_angle=min_angle, max_angle=max_angle )
        self.factor(factor)

    def factor(self, factor = None):
        if factor is not None:
            # 0 .. 90 (reverse) 90 .. 180 (forward)
            # center = ((min_angle + max_angle) /2)
            # range = (max_angle - center) 
            # newrange = int((range / 100) * MAX_ANGELS[self._factor]) 
            # reverse (center - newrange) 
            # forward (center + newrange)  
            range = int(((self._drive_max_angle - self._center) / 100) * MAX_ANGELS[factor])
            self._min_angle = int(self._center - range)
            self._max_angle = int(self._center + range)
     
    def stop(self):
        self._angle = self._center
        self._irq()


class RCBSI():
    def __init__(self):
        print("BSI")
        self.light = LIGHT(False)
        self.turn = SERVO( max_angle=135, min_angle=45)
        self.motor = DRIVE()
        self._handler = None
        self.motor.irq(handler=self._irq)
        self.turn.irq(handler=self._irq)
        self.light.irq(handler=self._irq)

    def irq(self, handler):
        self._handler = handler

    def _irq(self):
        if self._handler:
            self._handler()
    
    def playmobile_rc_msg(self, msg):
        # XX YY 0F
        # 0x23 - motor 00 (max backward) to 7f (stop) to ff (max forward)
        # 0x24 - light 01 (off), 02 (on) (does feedback to RC)
        # 0x25 - speed 01 to 05 (does feedback to RC)
        # 0x40 - turn 00 (max left) to 7f (straight) to ff (max right)
        #
        if len(msg) == 3:
            if msg[2] == 0x0F:
                command = msg[0]
                if command == 0x23:
                    self.motor(msg[1])
                elif command == 0x24:
                    self.light.set(msg[1])
                elif command ==  0x25:
                    self.motor.factor(msg[1])
                elif command ==  0x40:
                    self.turn(msg[1])
        else:
            pass   
    
    def on_connect(self, state):
        if state:
           print("Connected")
           self.light.off()
           self.turn.center()
           self.motor.stop()
           self.motor.factor(3)
        else:
            print("Disconnected")
            self.motor.stop()

        print("state:" + str(state))
        

def demo():
    import time
    ble = bluetooth.BLE()
    pmrc = BLEPMRC(ble)
    bsi = RCBSI()
    print("light: " + str(bsi.light))
    print("turn: " + str(int(bsi.turn)))
    print("motor: " + str(int(bsi.motor)))

    def on_rx():
        value = pmrc.read()
        pmrc.notify(value)
        bsi.playmobile_rc_msg(value)   

    pmrc.irq(handler=on_rx)

    i = 0

    def on_change():
        print("light: " + str(bsi.light))
        print("turn: " + str(int(bsi.turn)))
        print("motor: " + str(int(bsi.motor)))

    bsi.irq(handler=on_change)

    pmrc.connect(handler=bsi.on_connect)

    try:
        while True:
            #i = (i + 1) % len(nums)
            #print("i: " + str(i))
            time.sleep_ms(1000)
    except KeyboardInterrupt:
        pass

    pmrc.close()


if __name__ == "__main__":
    demo()
    timer.deinit()
