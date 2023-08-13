#include <iostream>
#include <opencv2/opencv.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>

void applySepia(cv::Mat &frame)
{
  cv::Mat temp = frame.clone();
  cv::transform(temp, frame, cv::Matx33f(
    0.272, 0.534, 0.131,
    0.349, 0.686, 0.168,
    0.393, 0.769, 0.189)
  );
}

void invertColors(cv::Mat &frame)
{
  cv::bitwise_not(frame, frame);
}

void applyGrayscale(cv::Mat &frame)
{
  cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
}

cv::Mat get_frame_from_sink(GstElement *sink, int &width, int &height)
{
  GstSample *sample = gst_app_sink_pull_sample(GST_APP_SINK(sink));

  if (!sample)
  {
    return cv::Mat();
  }

  if (width == 0 || height == 0)
  {
    GstCaps *caps = gst_sample_get_caps(sample);
    GstStructure *structure = gst_caps_get_structure(caps, 0);
    gst_structure_get_int(structure, "width", &width);
    gst_structure_get_int(structure, "height", &height);
  }

  GstBuffer *buffer = gst_sample_get_buffer(sample);
  GstMapInfo map_info;
  gst_buffer_map(buffer, &map_info, GST_MAP_READ);

  cv::Mat frame(height, width, CV_8UC4, map_info.data); // Dynamic resolution

  gst_buffer_unmap(buffer, &map_info);
  gst_sample_unref(sample);

  cv::cvtColor(frame, frame, cv::COLOR_BGRA2BGR);

  return frame;
}

int main(int argc, char *argv[])
{
  gst_init(&argc, &argv);

  GstElement *pipeline = gst_pipeline_new("video-pipeline");
  GstElement *source = gst_element_factory_make("v4l2src", "video-source");
  GstElement *convert = gst_element_factory_make("videoconvert", "converter");
  GstElement *sink = gst_element_factory_make("appsink", "video-sink");

  if (!pipeline || !source || !convert || !sink)
  {
    std::cerr << "Failed to create elements." << std::endl;
    return -1;
  }

  g_object_set(G_OBJECT(sink), "emit-signals", TRUE, "caps", gst_caps_from_string("video/x-raw, format=(string)BGRx"), NULL);
  gst_bin_add_many(GST_BIN(pipeline), source, convert, sink, NULL);
  gst_element_link_many(source, convert, sink, NULL);

  gst_element_set_state(pipeline, GST_STATE_PLAYING);

  int width = 0, height = 0;
  cv::Mat frame, gray_frame;

  while (true)
  {
    frame = get_frame_from_sink(sink, width, height); // Pass width and height
                                                      //
    if (frame.empty())
    {
      std::cerr << "Empty frame!" << std::endl;
      break;
    }

    //applySepia(frame);
    //invertColors(frame);
    applyGrayscale(frame);
    cv::imshow("Filtered Frame", frame);

    if (cv::waitKey(1) == 27)
    {
      break;
    }
  }

  gst_element_set_state(pipeline, GST_STATE_NULL);
  gst_object_unref(GST_OBJECT(pipeline));

  return 0;
}
