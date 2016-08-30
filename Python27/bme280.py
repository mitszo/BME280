#coding: utf-8

from smbus2 import SMBus


class BME280(object):
    def __init__(self, bus_number, i2c_address,
            osrs_t = 1,            #Temperature oversampling x 1
            osrs_p = 1,            #Pressure oversampling x 1
            osrs_h = 1,            #Humidity oversampling x 1
            mode   = 3,            #Normal mode
            t_sb   = 5,            #Tstandby 1000ms
            filter_ = 0,            #Filter off
            spi3w_en = 0,            #3-wire SPI Disable
            ):

        self.digT = []
        self.digP = []
        self.digH = []
        self.bus = SMBus(bus_number)
        self.i2c_address = i2c_address

        ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | mode
        config_reg    = (t_sb << 5) | (filter_ << 2) | spi3w_en
        ctrl_hum_reg  = osrs_h

        self.writeReg(0xF2,ctrl_hum_reg)
        self.writeReg(0xF4,ctrl_meas_reg)
        self.writeReg(0xF5,config_reg)

        self.get_calib_param()


    def writeReg(self, reg_address, data):
        self.bus.write_byte_data(self.i2c_address, reg_address, data)


    def read_byte_data(self, address):
        return self.bus.read_byte_data(self.i2c_address, address)


    def get_calib_param(self):
        calib = []
        digT = []
        digP = []
        digH = []

        for i in range (0x88,0x88+24):
            calib.append(self.read_byte_data(i))
        calib.append(self.read_byte_data(0xA1))
        for i in range (0xE1,0xE1+7):
            calib.append(self.read_byte_data(i))

        digT.append((calib[1] << 8) | calib[0])
        digT.append((calib[3] << 8) | calib[2])
        digT.append((calib[5] << 8) | calib[4])
        digP.append((calib[7] << 8) | calib[6])
        digP.append((calib[9] << 8) | calib[8])
        digP.append((calib[11]<< 8) | calib[10])
        digP.append((calib[13]<< 8) | calib[12])
        digP.append((calib[15]<< 8) | calib[14])
        digP.append((calib[17]<< 8) | calib[16])
        digP.append((calib[19]<< 8) | calib[18])
        digP.append((calib[21]<< 8) | calib[20])
        digP.append((calib[23]<< 8) | calib[22])
        digH.append( calib[24] )
        digH.append((calib[26]<< 8) | calib[25])
        digH.append( calib[27] )
        digH.append((calib[28]<< 4) | (0x0F & calib[29]))
        digH.append((calib[30]<< 4) | ((calib[29] >> 4) & 0x0F))
        digH.append( calib[31] )

        for i in range(1,2):
            if digT[i] & 0x8000:
                digT[i] = (-digT[i] ^ 0xFFFF) + 1

        for i in range(1,8):
            if digP[i] & 0x8000:
                digP[i] = (-digP[i] ^ 0xFFFF) + 1

        for i in range(0,6):
            if digH[i] & 0x8000:
                digH[i] = (-digH[i] ^ 0xFFFF) + 1

        self.digT = digT
        self.digP = digP
        self.digH = digH


    def readData(self):
        data = []
        for i in range (0xF7, 0xF7+8):
            data.append(self.read_byte_data(i))
        pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
        hum_raw  = (data[6] << 8)  |  data[7]

        temp = self.compensate_T(temp_raw)
        pres = self.compensate_P(pres_raw)
        hum = self.compensate_H(hum_raw)

        return temp, pres, hum


    def compensate_P(self, adc_P):
        t_fine = 0.0
        pressure = 0.0

        v1 = (t_fine / 2.0) - 64000.0
        v2 = (((v1 / 4.0) * (v1 / 4.0)) / 2048) * self.digP[5]
        v2 = v2 + ((v1 * self.digP[4]) * 2.0)
        v2 = (v2 / 4.0) + (self.digP[3] * 65536.0)
        v1 = (((self.digP[2] * (((v1 / 4.0) * (v1 / 4.0)) / 8192)) / 8)  + ((self.digP[1] * v1) / 2.0)) / 262144
        v1 = ((32768 + v1) * self.digP[0]) / 32768

        if v1 == 0:
            return 0
        pressure = ((1048576 - adc_P) - (v2 / 4096)) * 3125
        if pressure < 0x80000000:
            pressure = (pressure * 2.0) / v1
        else:
            pressure = (pressure / v1) * 2
        v1 = (self.digP[8] * (((pressure / 8.0) * (pressure / 8.0)) / 8192.0)) / 4096
        v2 = ((pressure / 4.0) * self.digP[7]) / 8192.0
        pressure = pressure + ((v1 + v2 + self.digP[6]) / 16.0)

        return pressure / 100

    def compensate_T(self, adc_T):
        v1 = (adc_T / 16384.0 - self.digT[0] / 1024.0) * self.digT[1]
        v2 = (adc_T / 131072.0 - self.digT[0] / 8192.0) * (adc_T / 131072.0 - self.digT[0] / 8192.0) * self.digT[2]
        t_fine = v1 + v2
        return t_fine / 5120.0

    def compensate_H(self, adc_H):
        t_fine = 0.0
        var_h = t_fine - 76800.0
        if var_h != 0:
            var_h = (adc_H - (self.digH[3] * 64.0 + self.digH[4]/16384.0 * var_h)) * (self.digH[1] / 65536.0 * (1.0 + self.digH[5] / 67108864.0 * var_h * (1.0 + self.digH[2] / 67108864.0 * var_h)))
        else:
            return 0
        var_h = var_h * (1.0 - self.digH[0] * var_h / 524288.0)
        if var_h > 100.0:
            var_h = 100.0
        elif var_h < 0.0:
            var_h = 0.0
        return var_h



if __name__ == '__main__':
    BUS_NUMBER  = 1
    I2C_ADDRESS = 0x76
    bme280 = BME280(BUS_NUMBER, I2C_ADDRESS)
    try:
        temperature, pressure, hum = bme280.readData()
        print "temp : %-6.2f ℃" % (temperature)
        print "pressure : %7.2f hPa" % (pressure)
        print "hum : %6.2f ％" % (hum)
    except KeyboardInterrupt:
        pass





