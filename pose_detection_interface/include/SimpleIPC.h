//
// Created by mmatsi on 07.05.21.
//

#ifndef POSE_DETECTION_INTERFACE_SIMPLEIPC_H
#define POSE_DETECTION_INTERFACE_SIMPLEIPC_H

#include <iostream>
#include <utility>
#include <semaphore.h>

class SimpleIPC {
private:
    std::string key;
    void *addr = nullptr;
    size_t mapping_size = 0;

    sem_t *sem = nullptr;


public:
    std::string error_message;

    explicit SimpleIPC(std::string shm_name) : key(std::move(shm_name)) {}

    bool create(size_t size);

    bool attach(size_t size);

    void *data();

    void detach();

    void lock();

    void unlock();

    ~SimpleIPC();
};


#endif //POSE_DETECTION_INTERFACE_SIMPLEIPC_H
