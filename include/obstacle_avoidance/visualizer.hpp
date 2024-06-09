// Created by yuwei on 11/14/19.

#ifndef SRC_VISUALIZATION_HPP
#define SRC_VISUALIZATION_HPP

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

using namespace std;

static int num_visuals = 0;

class PointsVisualizer {
protected:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub;
    visualization_msgs::msg::Marker dots;
    string ns;
    string frame_id;

public:
    PointsVisualizer(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub, string ns, string frame_id, std_msgs::msg::ColorRGBA color, float scale)
        : pub(pub), ns(ns), frame_id(frame_id) {
        dots.header.frame_id = frame_id;
        dots.ns = ns;
        dots.action = visualization_msgs::msg::Marker::ADD;
        dots.pose.orientation.w = 1.0;
        dots.id = num_visuals;
        dots.type = visualization_msgs::msg::Marker::POINTS;
        dots.scale.x = dots.scale.y = scale;
        dots.color = color;
        ++num_visuals;
    }

    void add_point(geometry_msgs::msg::Point p) {
        dots.points.push_back(p);
    }

    void publish_points() {
        pub->publish(dots);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Published dots");
        dots.points.clear();
    }
};

class MarkerVisualizer {
protected:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub;
    visualization_msgs::msg::Marker dot;
    string ns;
    string frame_id;

public:
    MarkerVisualizer(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub, string ns, string frame_id, std_msgs::msg::ColorRGBA color, float scale, int shape)
        : pub(pub), ns(ns), frame_id(frame_id) {
        dot.header.frame_id = frame_id;
        dot.ns = ns;
        dot.action = visualization_msgs::msg::Marker::ADD;
        dot.id = num_visuals;
        dot.type = shape;
        dot.scale.x = dot.scale.y = dot.scale.z = scale;
        dot.color = color;
        ++num_visuals;
    }

    void set_pose(geometry_msgs::msg::Pose pose) {
        dot.pose.orientation = pose.orientation;
        dot.pose.position = pose.position;
    }

    void publish_marker() {
        pub->publish(dot);
    }
};

class LineListVisualizer {
protected:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub;
    visualization_msgs::msg::Marker line_list;
    string ns;
    string frame_id;

public:
    LineListVisualizer(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub, string ns, string frame_id)
        : pub(pub), ns(ns), frame_id(frame_id) {
        line_list.header.frame_id = frame_id;
        line_list.ns = ns;
        line_list.action = visualization_msgs::msg::Marker::ADD;
        line_list.pose.orientation.w = 1.0;
        line_list.id = num_visuals;
        line_list.type = visualization_msgs::msg::Marker::LINE_LIST;
        line_list.scale.x = 0.01;
        line_list.color.r = 0.0; line_list.color.b = 1.0; line_list.color.g = 0.0; line_list.color.a = 1.0;
        ++num_visuals;
    }

    void add_line(geometry_msgs::msg::Point p1, geometry_msgs::msg::Point p2) {
        line_list.points.push_back(p1);
        line_list.points.push_back(p2);
    }

    void publish_line_list() {
        line_list.header.stamp = rclcpp::Time();
        pub->publish(line_list);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Published line list");
        line_list.points.clear();
    }
};

#endif // SRC_VISUALIZATION_HPP
