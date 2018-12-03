package com.example.ros_android_camera_controller;

import android.hardware.SensorManager;
import android.os.Bundle;
import android.view.WindowManager;

import org.ros.address.InetAddressFactory;
import org.ros.android.BitmapFromCompressedImage;
import org.ros.android.RosActivity;
import org.ros.android.view.RosImageView;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

public class MainActivity extends RosActivity {

    // Fields
    private RosImageView<sensor_msgs.CompressedImage> mImage;
    private OrientationPublisher mOrientationPublisher;
    private SensorManager mSensorManager;

    // Constructors
    public MainActivity() {
        super("Camera Controller",
                "Camera Controller");
    }

    // Protected Methods
    @SuppressWarnings("unchecked")
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        // Set up the sensor manager for the orientation data
        mSensorManager = (SensorManager)this.getSystemService(SENSOR_SERVICE);

        // Set up stuff for the image subscriber node
        mImage = (RosImageView<sensor_msgs.CompressedImage>) findViewById(R.id.image);
        mImage.setTopicName("webcam/image_raw/compressed");
        mImage.setMessageType(sensor_msgs.CompressedImage._TYPE);
        mImage.setMessageToBitmapCallable(new BitmapFromCompressedImage());
    }

    @Override
    protected void onPause() {
        super.onPause();
    }

    @Override
    protected void onResume() {
        super.onResume();
    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        // Sensor delay is 50 Hz
        int sensorDelay = 20000;

        // Compressed image subscriber node configuration
        NodeConfiguration imageSubscriberNodeConfiguration =
                NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
        imageSubscriberNodeConfiguration.setMasterUri(getMasterUri());
        imageSubscriberNodeConfiguration.setNodeName("compressed_image_subscriber");

        // Run the compressed image subscriber
        nodeMainExecutor.execute(mImage, imageSubscriberNodeConfiguration);

        // Orientation publisher node configuration
        NodeConfiguration orientationPublisherNodeConfiguration =
                NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
        orientationPublisherNodeConfiguration.setMasterUri(getMasterUri());
        orientationPublisherNodeConfiguration.setNodeName("orientation_publisher");

        // Create the publisher class
        this.mOrientationPublisher = new OrientationPublisher(mSensorManager, sensorDelay);

        // Run the orientation publisher
        nodeMainExecutor.execute(this.mOrientationPublisher, orientationPublisherNodeConfiguration);
    }
}
