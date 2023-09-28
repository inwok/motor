//Project Microbit Link plus LCD
//----------------------------------
// บล็อกคำสั่งสำหรับควบคุมมอเตอร์
enum MotorChannel {
    //% block="E"
    E,
    //% block="F"
    F,
    //% block="G"
    G,
    //% block="H"
    H,
}
enum MotorShaftDirection {
    //% block="Left"
    LOW,
    //% block="Right"
    HIGH,

}
let motorSpeedPins: { [key: number]: AnalogPin } = {
    [MotorChannel.E]: AnalogPin.P16,
    [MotorChannel.F]: AnalogPin.P14,
    [MotorChannel.G]: AnalogPin.P2,
    [MotorChannel.H]: AnalogPin.P8,
}
let motorChannels: { [key: number]: DigitalPin } = {
    [MotorChannel.E]: DigitalPin.P15,
    [MotorChannel.F]: DigitalPin.P13,
    [MotorChannel.G]: DigitalPin.P12,
    [MotorChannel.H]: DigitalPin.P1,
}

//----------------------------------
//led
enum LEDChannel {
    //% block="A"
    A,
    //% block="B"
    B,
    //% block="C"
    C,
    //% block="D"
    D,
    //% block="E"
    E,
    //% block="F"
    F,
    //% block="G"
    G,
    //% block="H"
    H,
}
let LEDChannels: { [key: number]: DigitalPin } = {
    [LEDChannel.A]: DigitalPin.P19,
    [LEDChannel.B]: DigitalPin.P14,
    [LEDChannel.C]: DigitalPin.P2,
    [LEDChannel.D]: DigitalPin.P8,
    [LEDChannel.E]: DigitalPin.P15,
    [LEDChannel.F]: DigitalPin.P13,
    [LEDChannel.G]: DigitalPin.P12,
    [LEDChannel.H]: DigitalPin.P1,
}
enum LEDShaftonoff {
    //% block="off"
    LOW,
    //% block="on"
    HIGH,

}
//----------------------------------
//button
enum buttonChannel {
    //% block="A"
    A,
    //% block="E"
    E,
    //% block="F"
    F,
    //% block="G"
    G,
    //% block="H"
    H,
}
let buttonChannels: { [key: number]: DigitalPin } = {
    [buttonChannel.A]: DigitalPin.P20,
    [buttonChannel.E]: DigitalPin.P16,
    [buttonChannel.F]: DigitalPin.P14,
    [buttonChannel.G]: DigitalPin.P2,
    [buttonChannel.H]: DigitalPin.P8,

}
//----------------------------------

//อ่านค่าเซ็นเซอร์
enum sensorChannel {
    //% block="P1"
    P1,
    //% block="P8"
    P8,
    //% block="P12"
    P12,
    //% block="P2"
    P2,
    //% block="P13"
    P13,
    //% block="P14"
    P14,
    //% block="P15"
    P15,
    //% block="P16"
    P16,
}
let sensorChannels: { [key: number]: DigitalPin } = {
    [sensorChannel.P1]: DigitalPin.P1,
    [sensorChannel.P8]: DigitalPin.P8,
    [sensorChannel.P12]: DigitalPin.P12,
    [sensorChannel.P2]: DigitalPin.P2,
    [sensorChannel.P13]: DigitalPin.P13,
    [sensorChannel.P14]: DigitalPin.P14,
    [sensorChannel.P15]: DigitalPin.P15,
    [sensorChannel.P16]: DigitalPin.P16,
}
//----------------------------------

//อ่านค่าเซ็นเซอร์
enum blackChannel {
    //% block="P1"
    P1,
    //% block="P8"
    P8,
    //% block="P12"
    P12,
    //% block="P2"
    P2,
    //% block="P13"
    P13,
    //% block="P14"
    P14,
    //% block="P15"
    P15,
    //% block="P16"
    P16,
}
let blackChannels: { [key: number]: DigitalPin } = {
    [blackChannel.P1]: DigitalPin.P1,
    [blackChannel.P8]: DigitalPin.P8,
    [blackChannel.P12]: DigitalPin.P12,
    [blackChannel.P2]: DigitalPin.P2,
    [blackChannel.P13]: DigitalPin.P13,
    [blackChannel.P14]: DigitalPin.P14,
    [blackChannel.P15]: DigitalPin.P15,
    [blackChannel.P16]: DigitalPin.P16,
}
//----------------------------------
//servo180
enum servoChannel {
    //% block="P1"
    P1,
    //% block="P8"
    P8,
    //% block="P12"
    P12,
    //% block="P2"
    P2,
    //% block="P13"
    P13,
    //% block="P14"
    P14,
    //% block="P15"
    P15,
    //% block="P16"
    P16,
}
let servoChannels: { [key: number]: AnalogPin } = {
    [servoChannel.P1]: AnalogPin.P1,
    [servoChannel.P8]: AnalogPin.P8,
    [servoChannel.P12]: AnalogPin.P12,
    [servoChannel.P2]: AnalogPin.P2,
    [servoChannel.P13]: AnalogPin.P13,
    [servoChannel.P14]: AnalogPin.P14,
    [servoChannel.P15]: AnalogPin.P15,
    [servoChannel.P16]: AnalogPin.P16,
}
//----------------------------------
//servoCon
enum servoconChannel {
    //% block="P1"
    P1,
    //% block="P8"
    P8,
    //% block="P12"
    P12,
    //% block="P2"
    P2,
    //% block="P13"
    P13,
    //% block="P14"
    P14,
    //% block="P15"
    P15,
    //% block="P16"
    P16,
}
let servoconChannels: { [key: number]: AnalogPin } = {
    [servoconChannel.P1]: AnalogPin.P1,
    [servoconChannel.P8]: AnalogPin.P8,
    [servoconChannel.P12]: AnalogPin.P12,
    [servoconChannel.P2]: AnalogPin.P2,
    [servoconChannel.P13]: AnalogPin.P13,
    [servoconChannel.P14]: AnalogPin.P14,
    [servoconChannel.P15]: AnalogPin.P15,
    [servoconChannel.P16]: AnalogPin.P16,
}
enum svconShaft {
    //% block="Right"
    Right = 0,
    //% block="Left"
    Left = 180,
    //% block="Stop"
    Stop = 90,

}
let degreesCon: { [key: number]: number } = {
    [svconShaft.Right]: 0,
    [svconShaft.Left]: 180,
    [svconShaft.Stop]: 90,

}
//----------------------------------
//sonar
enum PingUnit {
    //% block="μs"
    MicroSeconds,
    //% block="cm"
    Centimeters,
    //% block="inches"
    Inches
}
enum sonarPort {
    //% block="E"
    E,
    //% block="F"
    F,
    //% block="G"
    G,
    //% block="H"
    H,
}
let trigChanel: { [key: number]: DigitalPin } = {
    [sonarPort.E]: DigitalPin.P16,
    [sonarPort.F]: DigitalPin.P14,
    [sonarPort.G]: DigitalPin.P2,
    [sonarPort.H]: DigitalPin.P8,
}
let echoChanel: { [key: number]: DigitalPin } = {
    [sonarPort.E]: DigitalPin.P15,
    [sonarPort.F]: DigitalPin.P13,
    [sonarPort.G]: DigitalPin.P12,
    [sonarPort.H]: DigitalPin.P1,
}
//----------------------------------
//% color=#E7734B icon="\uf2db"
//% groups="['Motor','Servo','Led', 'Read Sensor','Logic Sensor','LCD i2c']"
namespace InwO {
        //สำหรับ motor

    //% color=#E7734B
    //% direction.defl=MotorShaftDirection.HIGH
    //% block="Stop Motor $channel"
    //% group="Motor"
    export function motorStop(channel: MotorChannel): void {
        let dirPin = motorChannels[channel];
        let speedPin = motorSpeedPins[channel];

        pins.digitalWritePin(dirPin, 0);
        pins.analogWritePin(speedPin, 0);
    }
    //% color=#E7734B
    //% block="Motor $channel direction $direction speed $speed"
    //% speed.min=0 speed.max=255
    //% direction.defl=MotorShaftDirection.HIGH
    //% group="Motor"
    //% color=#E7734B
    export function motorControl(channel: MotorChannel, direction: MotorShaftDirection, speed: number): void {
        let dirPin = motorChannels[channel];
        let speedPin = motorSpeedPins[channel];

        pins.digitalWritePin(dirPin, direction);
        pins.analogWritePin(speedPin, pins.map(speed, 0, 255, 0, 1023));
    }
    //% color=#E84E19
    //สำหรับ servo180
    //% block"servo180 $pinSmini degrees $degrees"
    //% degrees.min=20 degrees.max=160
    //% degrees.defl=90
    //% group="Servo"
    export function MiniServo(pinSmini: servoChannel, degrees: number): void {
        let pinsmini= servoChannels[pinSmini];
        pins.servoWritePin(pinsmini, degrees);

    }
    //% color=#E84E19
    //สำหรับ servocon
    //% block"ContinuousServo $pinSV direction $direction"
    //% direction.defl=90
    //% group="Servo"
    export function ContinuousServo(pinSV: servoChannel, direction: svconShaft): void {
        let pinservo = servoChannels[pinSV];
        pins.servoWritePin(pinservo, direction);

    }



    //% color=#FACB09
    //สำหรับ Led
    //% block="LED $leds Status $Status"
    //% Status.defl=LEDShaftonoff.HIGH*
    //% leds.defl=LEDChannel.D
    //% group="Led"
    export function led(leds: LEDChannel, Status: LEDShaftonoff): void {
        let ledg = LEDChannels[leds];

        pins.digitalWritePin(ledg, Status);

    }
    //% color=#FACB09
    //toggle led
    //% blockId=LED block="LED %pin $ledstate"
    //% ledstate.shadow="toggleOnOff"
    //% expandableArgumentMode="toggle"
    //% group="Led"
    export function ledBrightness(pin: LEDChannel, ledstate: boolean): void {
        if (ledstate) {
            let pinled = LEDChannels[pin];
            pins.digitalWritePin(pinled, 1);
           
        }
        else {
            let pinled = LEDChannels[pin];
            pins.digitalWritePin(pinled, 0);
           
        }
    }

    //% color=#000000
    //sonar
    //% block="sonar %channel unit %unit"
    //% group="Read Sensor"
    //% unit.defl=PingUnit.Centimeters
    export function ping(channel: sonarPort, unit: PingUnit, maxCmDistance = 500): number {
        let trig = trigChanel[channel];
        let echo = echoChanel[channel];
        // send pulse
        pins.setPull(trig, PinPullMode.PullNone);
        pins.digitalWritePin(trig, 0);
        control.waitMicros(2);
        pins.digitalWritePin(trig, 1);
        control.waitMicros(10);
        pins.digitalWritePin(trig, 0);

        // read pulse
        const d = pins.pulseIn(echo, PulseValue.High, maxCmDistance * 58);

        switch (unit) {
            case PingUnit.Centimeters: return Math.idiv(d, 58);
            case PingUnit.Inches: return Math.idiv(d, 148);
            default: return d;
        }
    }


    //% color=#3D3430
    //สำหรับ Track Line
    //% block="Track Line $pin Black Color"
    //% group="Logic Sensor"
    export function isButtonPressedII(pin: blackChannel): boolean {
        let read = blackChannels[pin];
        return pins.digitalReadPin(read) == 1;
    }
    //% color=#3D3430    
    //สำหรับ buttonpress
    //% block="On button $pin pressed"
    //% group="Logic Sensor"
    export function isButtonPressed(pin: buttonChannel): boolean {
        pins.setPull(buttonChannels[pin], PinPullMode.PullUp);
        let read = buttonChannels[pin];
        return pins.digitalReadPin(read) == 0;
    }
    //สำหรับ motion PIR3pin
    //% color=#3D3430   
    //% blockId=octopus_pir weight=80 blockGap=30
    //% block="motion detector at pin %p"    //% group="Logic Sensor"
    export function PIR(p: sensorChannel): boolean {
        let b = sensorChannels[p];
        let a: number = pins.digitalReadPin(b);
        if (a == 1) {
            return true;
        } else return false;
    }


    //% color=#000000
    //% block="Read button $pin (0-1)"
    //% group="Read Sensor"
    export function Readbutton(pin: buttonChannel): number {
        let read = buttonChannels[pin];
        pins.setPull(buttonChannels[pin], PinPullMode.PullUp);
        let reading = pins.digitalReadPin(read);
        return (reading);
    }
    //% color=#000000    
    //สำหรับ sensor
    //% block="Analog Sensor $pin (0-10) "
    //% group="Read Sensor"
    export function lightSensor(pin: sensorChannel): number {
        let read = servoconChannels[pin];
        let reading = pins.analogReadPin(read);
        let mappin = pins.map(reading, 0, 1023, 0, 10); // แปลงค่าจาก 0-1023 เป็น 0-10
        return Math.round(mappin);
    }

    //% color=#000000    
    //สำหรับ sensor
    //% block="Digital Sensor $pin (0-1)"
    //% group="Read Sensor"
    export function Sensor(pin: sensorChannel): number {
        let read = sensorChannels[pin];
        let reading = pins.digitalReadPin(read);
        return (reading);
    }

    //LCD i2c
    
    
    let i2cAddr: number // 0x3F: PCF8574A, 0x27: PCF8574
    let BK: number      // backlight control
    let RS: number      // command/data

    // set LCD reg
    function setreg(d: number) {
        pins.i2cWriteNumber(i2cAddr, d, NumberFormat.Int8LE)
        basic.pause(1)
    }

    // send data to I2C bus
    function set(d: number) {
        d = d & 0xF0
        d = d + BK + RS
        setreg(d)
        setreg(d + 4)
        setreg(d)
    }

    // send command
    function cmd(d: number) {
        RS = 0
        set(d)
        set(d << 4)
    }

    // send data
    function dat(d: number) {
        RS = 1
        set(d)
        set(d << 4)
    }

    // auto get LCD address
    function AutoAddr() {
        let k = true
        let addr = 0x20
        let d1 = 0, d2 = 0
        while (k && (addr < 0x28)) {
            pins.i2cWriteNumber(addr, -1, NumberFormat.Int32LE)
            d1 = pins.i2cReadNumber(addr, NumberFormat.Int8LE) % 16
            pins.i2cWriteNumber(addr, 0, NumberFormat.Int16LE)
            d2 = pins.i2cReadNumber(addr, NumberFormat.Int8LE)
            if ((d1 == 7) && (d2 == 0)) k = false
            else addr += 1
        }
        if (!k) return addr

        addr = 0x38
        while (k && (addr < 0x40)) {
            pins.i2cWriteNumber(addr, -1, NumberFormat.Int32LE)
            d1 = pins.i2cReadNumber(addr, NumberFormat.Int8LE) % 16
            pins.i2cWriteNumber(addr, 0, NumberFormat.Int16LE)
            d2 = pins.i2cReadNumber(addr, NumberFormat.Int8LE)
            if ((d1 == 7) && (d2 == 0)) k = false
            else addr += 1
        }
        if (!k) return addr
        else return 0

    }
    
    
    //% color=#045F14
    //% blockId="I2C_LCD1620_SET_ADDRESS" block="LCD (A) Address %addr"
    //% weight=100 blockGap=8
    //% parts=LCD1602_I2C trackArgs=0
    //% group="LCD i2c"
    export function LcdInit(Addr: number) {
        if (Addr == 0) i2cAddr = AutoAddr()
        else i2cAddr = Addr
        BK = 8
        RS = 0
        cmd(0x33)       // set 4bit mode
        basic.pause(5)
        set(0x30)
        basic.pause(5)
        set(0x20)
        basic.pause(5)
        cmd(0x28)       // set mode
        cmd(0x0C)
        cmd(0x06)
        cmd(0x01)       // clear
    }

    //% color=#045F14
    //% blockId="I2C_LCD1620_SHOW_NUMBER" block="show number %n|at x %x|y %y"
    //% weight=90 blockGap=8
    //% x.min=0 x.max=15
    //% y.min=0 y.max=1
    //% parts=LCD1602_I2C trackArgs=0
    //% group="LCD i2c"
    export function ShowNumber(n: number, x: number, y: number): void {
        let s = n.toString()
        ShowString(s, x, y)
    }

    //% color=#045F14
    //% blockId="I2C_LCD1620_SHOW_STRING" block="show string %s|at x %x|y %y"
    //% weight=90 blockGap=8
    //% x.min=0 x.max=15
    //% y.min=0 y.max=1
    //% parts=LCD1602_I2C trackArgs=0
    //% group="LCD i2c"
    export function ShowString(s: string, x: number, y: number): void {
        let a: number

        if (y > 0)
            a = 0xC0
        else
            a = 0x80
        a += x
        cmd(a)

        for (let i = 0; i < s.length; i++) {
            dat(s.charCodeAt(i))
        }
    }

    //% color=#045F14
    //% blockId="I2C_LCD1620_ON" block="LCD on"
    //% weight=81 blockGap=8
    //% parts=LCD1602_I2C trackArgs=0
    //% group="LCD i2c"
    export function on(): void {
        cmd(0x0C)
    }

    //% color=#045F14
    //% blockId="I2C_LCD1620_OFF" block="LCD off"
    //% weight=80 blockGap=8
    //% parts=LCD1602_I2C trackArgs=0
    //% group="LCD i2c"
    export function off(): void {
        cmd(0x08)
    }

    //% color=#045F14
    //% blockId="I2C_LCD1620_CLEAR" block="clear"
    //% weight=85 blockGap=8
    //% parts=LCD1602_I2C trackArgs=0
    //% group="LCD i2c"
    export function clear(): void {
        cmd(0x01)
    }

    //% color=#045F14
    //% blockId="I2C_LCD1620_BACKLIGHT_ON" block="light on"
    //% weight=71 blockGap=8
    //% parts=LCD1602_I2C trackArgs=0
    //% group="LCD i2c"
    export function BacklightOn(): void {
        BK = 8
        cmd(0)
    }

    //% color=#045F14
    //% blockId="I2C_LCD1620_BACKLIGHT_OFF" block="light off "
    //% weight=70 blockGap=8
    //% parts=LCD1602_I2C trackArgs=0
    //% group="LCD i2c"
    export function BacklightOff(): void {
        BK = 0
        cmd(0)
    }



}







