#include "NexDynIPC/App/StateExporter.hpp"
#include <iostream>
#include <fstream>
#include <filesystem>

namespace NexDynIPC::App {

StateExporter::StateExporter(const std::string& output_dir) : output_dir_(output_dir) {
    std::filesystem::create_directories(output_dir_);
}

void StateExporter::exportFrame(const Dynamics::World& world, int frame_idx, double time) {
    std::string filename = output_dir_ + "/simulation_results.csv";
    std::ofstream out;

    // Open in append mode
    // If it's the first frame (frame_idx == 0), we might want to overwrite or handle header.
    // For simplicity, let's assume if frame_idx == 0 we overwrite, else append.
    if (frame_idx == 0) {
        out.open(filename);
        // Write header
        out << "frame_idx,time,body_id,body_name,pos_x,pos_y,pos_z,qx,qy,qz,qw,vx,vy,vz,ax,ay,az,wx,wy,wz,alphax,alphay,alphaz\n";
    } else {
        out.open(filename, std::ios_base::app);
    }

    /*
    CSV Columns:
    frame_idx, time, body_id, body_name, 
    pos_x, pos_y, pos_z, 
    qx, qy, qz, qw, 
    vx, vy, vz, 
    ax, ay, az, 
    wx, wy, wz, 
    alphax, alphay, alphaz
    */

    for (const auto& body : world.bodies) {
        out << frame_idx << ","
            << time << ","
            << body->id << ","
            << body->name << ","
            << body->position.x() << "," << body->position.y() << "," << body->position.z() << ","
            << body->orientation.x() << "," << body->orientation.y() << "," << body->orientation.z() << "," << body->orientation.w() << ","
            << body->velocity.x() << "," << body->velocity.y() << "," << body->velocity.z() << ","
            << body->linear_acceleration.x() << "," << body->linear_acceleration.y() << "," << body->linear_acceleration.z() << ","
            << body->angular_velocity.x() << "," << body->angular_velocity.y() << "," << body->angular_velocity.z() << ","
            << body->angular_acceleration.x() << "," << body->angular_acceleration.y() << "," << body->angular_acceleration.z() << "\n";
    }
    out.close();
}

} // namespace NexDynIPC::App
