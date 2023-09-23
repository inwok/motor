//Project Microbit Link
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
//% color=#001365 icon="\uf2db"
//% groups="['Motor','Servo','Led', 'Read Sensor','Logic Sensor','LCD i2c']"
namespace InwO {
        //สำหรับ motor


    //% direction.defl=MotorShaftDirection.HIGH
    //% block="Stop Motor $channel"
    //% group="Motor"
    export function motorStop(channel: MotorChannel): void {
        let dirPin = motorChannels[channel];
        let speedPin = motorSpeedPins[channel];

        pins.digitalWritePin(dirPin, 0);
        pins.analogWritePin(speedPin, 0);
    }
    //% block="Motor $channel direction $direction speed $speed"
    //% speed.min=0 speed.max=255
    //% direction.defl=MotorShaftDirection.HIGH
    //% group="Motor"

    export function motorControl(channel: MotorChannel, direction: MotorShaftDirection, speed: number): void {
        let dirPin = motorChannels[channel];
        let speedPin = motorSpeedPins[channel];

        pins.digitalWritePin(dirPin, direction);
        pins.analogWritePin(speedPin, pins.map(speed, 0, 255, 0, 1023));
    }
    //สำหรับ servo180
    //% block"servo180 $pinSmini degrees $degrees"
    //% degrees.min=20 degrees.max=160
    //% degrees.defl=90
    //% group="Servo"
    export function MiniServo(pinSmini: servoChannel, degrees: number): void {
        let pinsmini= servoChannels[pinSmini];
        pins.servoWritePin(pinsmini, degrees);

    }

    //สำหรับ servocon
    //% block"ContinuousServo $pinSV direction $direction"
    //% direction.defl=90
    //% group="Servo"
    export function ContinuousServo(pinSV: servoChannel, direction: svconShaft): void {
        let pinservo = servoChannels[pinSV];
        pins.servoWritePin(pinservo, direction);

    }




    //สำหรับ Led
    //% block="LED $leds Status $Status"
    //% Status.defl=LEDShaftonoff.HIGH*
    //% leds.defl=LEDChannel.C
    //% group="Led"
    export function led(leds: LEDChannel, Status: LEDShaftonoff): void {
        let ledg = LEDChannels[leds];

        pins.digitalWritePin(ledg, Status);

    }

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



    //สำหรับ Track Line
    //% block="Track Line $pin Black Color"
    //% group="Logic Sensor"
    export function isButtonPressedII(pin: blackChannel): boolean {
        let read = blackChannels[pin];
        return pins.digitalReadPin(read) == 1;
    }
    //สำหรับ buttonpress
    //% block="On button $pin pressed"
    //% group="Logic Sensor"
    export function isButtonPressed(pin: buttonChannel): boolean {
        pins.setPull(buttonChannels[pin], PinPullMode.PullUp);
        let read = buttonChannels[pin];
        return pins.digitalReadPin(read) == 0;
    }


    //% block="Read button $pin "
    //% group="Read Sensor"
    export function Readbutton(pin: buttonChannel): number {
        let read = buttonChannels[pin];
        pins.setPull(buttonChannels[pin], PinPullMode.PullUp);
        let reading = pins.digitalReadPin(read);
        return (reading);
    }
    //สำหรับ sensor
    //% block"Sensor $pin "
    //% group="Read Sensor"
    export function Sensor(pin: sensorChannel): number {
        let read = sensorChannels[pin];
        let reading = pins.digitalReadPin(read);
        return (reading);
    }

    //LCD i2c
    //% block="LCD coming soon!"
    //% group="LCD i2c"
     export function lcdcommingsoon(): void {
        
       
    }


}






