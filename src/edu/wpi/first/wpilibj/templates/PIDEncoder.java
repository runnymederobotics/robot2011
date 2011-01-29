package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.*;

//PIDEncoder inherits PIDSource
public class PIDEncoder implements PIDSource {
    Encoder encoder;
    boolean rate;

    public PIDEncoder(boolean rateEncoder, int aChannel, int bChannel, boolean reverseDirection, Encoder.EncodingType encodingType) {
        encoder = new Encoder(aChannel, bChannel, reverseDirection, encodingType);
        rate = rateEncoder;
    }

    //Overloaded constructor where encodingType has a default value of k1X
    public PIDEncoder(boolean rateEncoder, int aChannel, int bChannel, boolean reverseDirection) {
        this(rateEncoder, aChannel, bChannel, reverseDirection, Encoder.EncodingType.k1X);
    }

    //Overloaded constructor where reverseDirection has a default value of false and encodingType has a default value of k1X
    public PIDEncoder(boolean rateEncoder, int aChannel, int bChannel) {
        this(rateEncoder, aChannel, bChannel, false, Encoder.EncodingType.k1X);
    }

    //Start the encoder
    public void start() {
        encoder.start();
    }

    //Reset the encoder
    public void reset() {
        encoder.reset();
    }

    //Function must be overrided, and the return value is the rate of the encoder
    public double pidGet() {
        if(rate) {
            return encoder.getRate();
        }
        else {
            return encoder.get();
        }
    }
}
