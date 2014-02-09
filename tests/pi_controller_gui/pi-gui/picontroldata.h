#ifndef DEF_PICONTROLDATA_H
#define DEF_PICONTROLDATA_H

struct PIControlData {
    unsigned int time;
    unsigned int dutyCycle;
    unsigned int temperature;

    float getDutyCycle(void) const {

        return (dutyCycle / 255.0f);
    }

    float getTime(void) const {
        return (time / 1000.0f);
    }

    float getTemperature(const unsigned int sensor) const {

        if(sensor > 2) {
            return -1.0f;
        }

        return ((temperature * 70.0f / 1024.0f) - 10.0f);
    }
};

#endif  /* DEF_PICONTROLDATA_H */
