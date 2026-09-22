#!/usr/bin/env python3
import subprocess
from gpiozero import LED

led1 = None
led2 = None
led3 = None

def gpioinfo_line(gpio):
    try:
        result = subprocess.run(
            ["gpioinfo"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=2
        )

        if result.returncode != 0:
            return None

        gpio_name = f'"GPIO{gpio}"'

        for line in result.stdout.splitlines():
            if gpio_name in line:
                return line.strip()
    except Exception:
        pass
    return None

def gpio_kernel_busy(gpio):
    line = gpioinfo_line(gpio)

    if line is None:
        return False

    if 'consumer="kernel"' in line or 'consumer="spi' in line:
        return True
    else:    
        return False

def create_led(gpio):
    if gpio_kernel_busy(gpio):
        print(f"GPIO{gpio} is occupied by kernel/SPI, LED disabled")
        return None
    try:
        led = LED(gpio)
        return led
    except Exception as e:
        pass

def switchSetup():
    global led1, led2, led3
    led1 = create_led(9)
    led2 = create_led(25)
    led3 = create_led(11)

def switch(port, status):
    if port == 1:
        if led1 is None:
            return

        if status == 1:
            led1.on()
        elif status == 0:
            led1.off()

    elif port == 2:
        if led2 is None:
            return

        if status == 1:
            led2.on()
        elif status == 0:
            led2.off()

    elif port == 3:
        if led3 is None:
            return

        if status == 1:
            led3.on()
        elif status == 0:
            led3.off()

def set_all_switch_off():
    switch(1, 0)
    switch(2, 0)
    switch(3, 0)

def switchClose():
    global led1, led2, led3

    if led1 is not None:
        try:
            led1.close()
        except Exception:
            pass

    if led2 is not None:
        try:
            led2.close()
        except Exception:
            pass

    if led3 is not None:
        try:
            led3.close()
        except Exception:
            pass

    led1 = None
    led2 = None
    led3 = None

if __name__ == "__main__":
    import time

    switchSetup()

    try:
        while True:
            switch(1, 1)
            time.sleep(1)

            switch(2, 1)
            time.sleep(1)

            switch(3, 1)
            time.sleep(1)

            set_all_switch_off()
            time.sleep(1)

    except KeyboardInterrupt:
        pass

    finally:
        set_all_switch_off()
        switchClose()