package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.parsing.ISensor;

public class Gyro1310 extends SensorBase implements PIDSource, ISensor {

    static final int kOversampleBits = 10;
    static final int kAverageBits = 0;
    static final double kSamplesPerSecond = 50.0;
    static final double kCalibrationSampleTime = 5.0;
    static final double kDefaultVoltsPerDegreePerSecond = 0.007;
    AnalogChannel m_analog;
    double m_voltsPerDegreePerSecond;
    double m_offset;
    boolean m_channelAllocated;
    AccumulatorResult result;

    public long min = 1000000000;
    public long max = 0;
    public int deadzone = 0;
    public int center = 0;

    /**
     * Initialize the gyro.
     * Calibrate the gyro by running for a number of samples and computing the center value for this
     * part. Then use the center value as the Accumulator center value for subsequent measurements.
     * It's important to make sure that the robot is not moving while the centering calculations are
     * in progress, this is typically done when the robot is first turned on while it's sitting at
     * rest before the competition starts.
     */
    private void initGyro() {
        result = new AccumulatorResult();
        if (m_analog == null) {
            System.out.println("Null m_analog");
        }
        m_voltsPerDegreePerSecond = kDefaultVoltsPerDegreePerSecond;
        m_analog.setAverageBits(kAverageBits);
        m_analog.setOversampleBits(kOversampleBits);
        double sampleRate = kSamplesPerSecond * (1 << (kAverageBits + kOversampleBits));
        m_analog.getModule().setSampleRate(sampleRate);

        Timer.delay(1.0);
        m_analog.initAccumulator();

        Timer.delay(kCalibrationSampleTime);

        double start = Timer.getFPGATimestamp();
        AccumulatorResult temp = new AccumulatorResult();
        while(Timer.getFPGATimestamp() - start < kCalibrationSampleTime * 0.25) {
            m_analog.getAccumulatorOutput(temp);
            result.count += temp.count;
            result.value += temp.value;
            long avg = (long) ((double)temp.value / (double)temp.count + .5);
            if(avg < min)
                min = avg;
            if(avg > max)
                max = avg;
            m_analog.resetAccumulator();
        }
        //Timer.delay(kCalibrationSampleTime);

        deadzone = (int)(max - min);

        m_analog.getAccumulatorOutput(result);

        center = (int) ((double)result.value / (double)result.count + .5);

        m_offset = ((double)result.value / (double)result.count) - (double)center;

        m_analog.setAccumulatorCenter(center);

        m_analog.setAccumulatorDeadband(0); ///< TODO: compute / parameterize this
        m_analog.resetAccumulator();
    }

    /**
     * Gyro constructor given a slot and a channel.
    .
     * @param slot The cRIO slot for the analog module the gyro is connected to.
     * @param channel The analog channel the gyro is connected to.
     */
    public Gyro1310(int slot, int channel) {
        m_analog = new AnalogChannel(slot, channel);
        m_channelAllocated = true;
        initGyro();
    }

    /**
     * Gyro constructor with only a channel.
     *
     * Use the default analog module slot.
     *
     * @param channel The analog channel the gyro is connected to.
     */
    public Gyro1310(int channel) {
        m_analog = new AnalogChannel(channel);
        m_channelAllocated = true;
        initGyro();
    }

    /**
     * Gyro constructor with a precreated analog channel object.
     * Use this constructor when the analog channel needs to be shared. There
     * is no reference counting when an AnalogChannel is passed to the gyro.
     * @param channel The AnalogChannel object that the gyro is connected to.
     */
    public Gyro1310(AnalogChannel channel) {
        m_analog = channel;
        if (m_analog == null) {
            System.err.println("Analog channel supplied to Gyro constructor is null");
        } else {
            m_channelAllocated = false;
            initGyro();
        }
    }

    /**
     * Reset the gyro.
     * Resets the gyro to a heading of zero. This can be used if there is significant
     * drift in the gyro and it needs to be recalibrated after it has been running.
     */
    public void reset() {
        if (m_analog != null) {
            m_analog.resetAccumulator();
        }
    }

    /**
     * Delete (free) the accumulator and the analog components used for the gyro.
     */
    protected void free() {
        if (m_analog != null && m_channelAllocated) {
            //m_analog.free();
        }
        m_analog = null;
    }

    /**
     * Return the actual angle in degrees that the robot is currently facing.
     *
     * The angle is based on the current accumulator value corrected by the oversampling rate, the
     * gyro type and the A/D calibration values.
     * The angle is continuous, that is can go beyond 360 degrees. This make algorithms that wouldn't
     * want to see a discontinuity in the gyro output as it sweeps past 0 on the second time around.
     *
     * @return the current heading of the robot in degrees. This heading is based on integration
     * of the returned rate from the gyro.
     */
    public double getAngle() {
        if (m_analog == null) {
            return 0.0;
        } else {
            m_analog.getAccumulatorOutput(result);

            long value = result.value - (long) (result.count * m_offset);

            double scaledValue = value * 1e-9 * m_analog.getLSBWeight() * (1 << m_analog.getAverageBits()) /
                    (m_analog.getModule().getSampleRate() * m_voltsPerDegreePerSecond);

            return scaledValue;
        }
    }

    /**
     * Set the gyro type based on the sensitivity.
     * This takes the number of volts/degree/second sensitivity of the gyro and uses it in subsequent
     * calculations to allow the code to work with multiple gyros.
     *
     * @param voltsPerDegreePerSecond The type of gyro specified as the voltage that represents one degree/second.
     */
    public void setSensitivity(double voltsPerDegreePerSecond) {
        m_voltsPerDegreePerSecond = voltsPerDegreePerSecond;
    }

    /**
     * Get the angle of the gyro for use with PIDControllers
     * @return the current angle according to the gyro
     */
    public double pidGet() {
        return getAngle();
    }
}