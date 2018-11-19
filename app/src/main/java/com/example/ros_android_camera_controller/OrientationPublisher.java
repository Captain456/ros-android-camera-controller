package com.example.ros_android_camera_controller;

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

import geometry_msgs.Quaternion;

public class OrientationPublisher implements NodeMain {

    // Fields
    private Sensor mSensor;
    private SensorManager mSensorManager;
    private int mSensorDelay;
    private OrientationListenerThread mOrientationListenerThread;
    private Publisher<Quaternion> mPublisher;

    // Classes
    private class OrientationListenerThread extends Thread {

        //Fields
        private final SensorManager mSensorManager;
        private OrientationEventListener mOrienationEventListener;
        private Looper mLooper;
        private final Sensor mOrientationSensor;

        // Constructors
        private OrientationListenerThread(
                SensorManager sensorManager,
                OrientationEventListener orientationEventListener) {
            this.mSensorManager = sensorManager;
            this.mOrienationEventListener = orientationEventListener;
            // Set up to listen for orientation events
            this.mOrientationSensor = this.mSensorManager
                    .getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);
        }

        public void run() {
            // Initialize the current thread as a Looper for handling messages
            Looper.prepare();
            // myLooper() returns the current thread's Looper object
            this.mLooper = Looper.myLooper();
            // Register the orientation listener
            this.mSensorManager.registerListener(
                    this.mOrienationEventListener,
                    this.mOrientationSensor,
                    mSensorDelay);
            // Loop to handle messages
            Looper.loop();
        }

        public void shutdown() {
            // Unregister the orientation listener
            this.mSensorManager.unregisterListener(this.mOrienationEventListener);

            // If the Looper has not been cleaned up, stop it
            if (this.mLooper != null) {
                this.mLooper.quit();
            }
        }
    }

    private class OrientationEventListener implements SensorEventListener {

        // Fields
        private Publisher<Quaternion> mPublisher;
        private float[] mOrientationQuaternion = new float[4];

        // Constructors
        private OrientationEventListener(Publisher<Quaternion> publisher) {
            this.mPublisher = publisher;
        }

        // Public Methods
        @Override
        public void onAccuracyChanged(Sensor sensor, int accuracy) {
            // Do nothing
        }

        @Override
        public void onSensorChanged(SensorEvent sensorEvent) {
            if (sensorEvent.sensor.getType() == Sensor.TYPE_ROTATION_VECTOR)
            {
                // Get the quaternion from the SensorManager
                SensorManager.getQuaternionFromVector(
                        this.mOrientationQuaternion,
                        sensorEvent.values);

                this.publishOrientationMessage();
            }
        }

        // Private Methods
        private void publishOrientationMessage() {
            // Create a new message to publish
            geometry_msgs.Quaternion message = this.mPublisher.newMessage();

            // Set the quaternion values of the message
            message.setW(this.mOrientationQuaternion[0]);
            message.setX(this.mOrientationQuaternion[1]);
            message.setY(this.mOrientationQuaternion[2]);
            message.setZ(this.mOrientationQuaternion[3]);

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
            this.mPublisher = node.newPublisher("orientation", "geometry_msgs/Quaternion");

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

}
