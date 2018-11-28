package com.example.ros_android_camera_controller;

import android.hardware.GeomagneticField;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Looper;

import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;

import std_msgs.Float32;

public class OrientationPublisher implements NodeMain {

    // Fields
    private Sensor mSensor;
    private SensorManager mSensorManager;
    private int mSensorDelay;
    private OrientationListenerThread mOrientationListenerThread;
    private Publisher<Float32> mPublisher;
    private float[] gravity = new float[3];
    private float[] geomagnetic = new float[3];
    private float[] rotation = new float[9];
    private float[] orientation = new float[3];
    private float[] smoothened_values = new float[3];

    // Classes
    private class OrientationListenerThread extends Thread {

        //Fields
        private final SensorManager mSensorManager;
        private OrientationEventListener mOrienationEventListener;
        private Looper mLooper;
        private final Sensor mAccelerometerSensor;
        private final Sensor mMagneticFieldSensor;

        // Constructors
        private OrientationListenerThread(
                SensorManager sensorManager,
                OrientationEventListener orientationEventListener) {
            this.mSensorManager = sensorManager;
            this.mOrienationEventListener = orientationEventListener;
            // Set up to listen for orientation events
            this.mAccelerometerSensor = this.mSensorManager
                    .getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
            this.mMagneticFieldSensor = this.mSensorManager
                    .getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        }

        public void run() {
            // Initialize the current thread as a Looper for handling messages
            Looper.prepare();
            // myLooper() returns the current thread's Looper object
            this.mLooper = Looper.myLooper();
            // Register the orientation listeners
            this.mSensorManager.registerListener(
                    this.mOrienationEventListener,
                    this.mAccelerometerSensor,
                    mSensorDelay);
            this.mSensorManager.registerListener(
                    this.mOrienationEventListener,
                    this.mMagneticFieldSensor,
                    mSensorDelay);
            // Loop to handle messages
            Looper.loop();
        }

        public void shutdown() {
            // Unregister the orientation listeners
            this.mSensorManager.unregisterListener(
                    this.mOrienationEventListener,
                    this.mAccelerometerSensor);
            this.mSensorManager.unregisterListener(
                    this.mOrienationEventListener,
                    this.mMagneticFieldSensor);

            // If the Looper has not been cleaned up, stop it
            if (this.mLooper != null) {
                this.mLooper.quit();
            }
        }
    }

    private class OrientationEventListener implements SensorEventListener {

        // Fields
        private Publisher<Float32> mPublisher;
        private float mBearingAngle = 0.0f;
        private float mStartingAngle = 0.0f;
        private int mEventCount = 0;
        private boolean mHasStartingAngle = false;

        // Constructors
        private OrientationEventListener(Publisher<Float32> publisher) {
            this.mPublisher = publisher;
        }

        // Public Methods
        @Override
        public void onAccuracyChanged(Sensor sensor, int accuracy) {
            // Do nothing
        }

        @Override
        public void onSensorChanged(SensorEvent sensorEvent) {
            boolean desiredSensor = false;

            if (sensorEvent.sensor.getType() == Sensor.TYPE_ACCELEROMETER)
            {
                // Smoothen the data via a low pass filter
                smoothened_values = lowPassFilter(sensorEvent.values, gravity);

                // Change the values of gravity to the smoothened values
                for (int i = 0; i < gravity.length; i++) {
                    gravity[i] = smoothened_values[i];
                }

                desiredSensor = true;
            }
            else if (sensorEvent.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
                // Smoothen the data via a low pass filter
                smoothened_values = lowPassFilter(sensorEvent.values, geomagnetic);

                // Change the values of geomagnetic to the smoothened values
                for (int i = 0; i < geomagnetic.length; i++) {
                    geomagnetic[i] = smoothened_values[i];
                }

                desiredSensor = true;
            }

            if (desiredSensor) {
                // Get the rotation and orientation matrices
                SensorManager.getRotationMatrix(rotation, null, gravity, geomagnetic);
                SensorManager.getOrientation(rotation, orientation);

                if (mEventCount == 20) {
                    if (!mHasStartingAngle) {
                        // We have received enough messages to be confident that the orientation is
                        // now accurate. Set the starting angle.
                        mStartingAngle = (float) Math.toDegrees(orientation[0]);
                        mHasStartingAngle = true;
                    }

                    // May need to mess with which orientation index to use
                    // UPDATE: This is the correct index
                    mBearingAngle = (float) Math.toDegrees(orientation[0]) - mStartingAngle + 90;

                    // Make sure that the bearing is between -90 and 270 degrees
                    if (mBearingAngle < -90) {
                        mBearingAngle += 360;
                    }
                    else if (mBearingAngle >= 270) {
                        mBearingAngle -= 360;
                    }

                    // Publish the bearing angle in a ROS message
                    this.publishOrientationMessage();
                }
                else {
                    mEventCount++;
                }
            }
        }

        // Private Methods
        private void publishOrientationMessage() {
            // Create a new message to publish
            std_msgs.Float32 message = this.mPublisher.newMessage();

            // Set the value of the message to be the bearing angle
            message.setData(mBearingAngle);

            // Publish
            this.mPublisher.publish(message);
        }
    }

    // Constructors
    public OrientationPublisher(SensorManager sensorManager, int sensorDelay) {
        this.mSensorManager = sensorManager;
        this.mSensorDelay = sensorDelay;
    }

    // Public Methods
    public void onStart(ConnectedNode node) {
        try {
            // Create the ROS publisher
            this.mPublisher = node.newPublisher("orientation", "std_msgs/Float32");

            // Create the OrientationEventListener
            OrientationEventListener orientationEventListener =
                    new OrientationEventListener(this.mPublisher);

            // Create and run the thread for listening to orientation events and publishing messages
            this.mOrientationListenerThread = new OrientationListenerThread(
                    this.mSensorManager,
                    orientationEventListener);
            this.mOrientationListenerThread.start();
        }
        catch (Exception exception) {
            if (node != null) {
                node.getLog().fatal(exception);
            }
            else {
                exception.printStackTrace();
            }
        }
    }

    public void onShutdown(Node node) {
        // Check if the thread is already shut down
        if (this.mOrientationListenerThread != null) {
            // If not, shut it down
            this.mOrientationListenerThread.shutdown();

            try {
                // Force the current thread to wait until mOrientationListenerThread is done
                this.mOrientationListenerThread.join();
            }
            catch (InterruptedException exception) {
                // Something went wrong. Print the exception to the stack trace
                exception.printStackTrace();
            }
        }
    }

    public void onShutdownComplete(Node node) {
        // Do nothing
    }

    public void onError(Node node, Throwable throwable) {

    }

    public GraphName getDefaultNodeName() {
        return GraphName.of("camera_controller/orientationPublisher");
    }

    // Protected Methods
    protected float[] lowPassFilter(float[] in, float[] out)
    {
        float alpha = 0.25f;

        if (out == null) {
            return in;
        }

        // Apply the low pass filter
        for (int i = 0; i < in.length; i++) {
            out[i] = out[i] + alpha * (in[i] - out[i]);
        }

        return out;
    }
}
