//
// Created by mmatsi on 07.05.21.
//

#include "SimpleIPC.h"
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>

bool SimpleIPC::create(size_t size) {
    int fd = shm_open(key.c_str(), O_RDWR | O_CREAT | O_EXCL, 0666);
    if (fd == -1) {
        error_message = "Error while creating shared memory object\n";
        return false;
    }

    ftruncate(fd, size);

    addr = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (addr == MAP_FAILED) {
        error_message = "Error wile mapping a file\n";
        return false;
    }
    mapping_size = size;

    sem = sem_open(key.c_str(), O_RDWR | O_CREAT, S_IRWXU | S_IRWXG | S_IRWXO, 1);
    if (sem == SEM_FAILED) {
        error_message = "Error while opening a semaphor\n";
        return false;
    }

    close(fd);
    return true;
}

bool SimpleIPC::attach(size_t size) {
    int fd = shm_open(key.c_str(), O_RDWR, 0666);
    if (fd == -1) {
        error_message = "Error while opening shared memory object\n";
        return false;
    }

    addr = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (addr == MAP_FAILED) {
        error_message = "Error wile mapping a file\n";
        return false;
    }
    mapping_size = size;

    sem = sem_open(key.c_str(), O_RDWR);
    if (sem == SEM_FAILED) {
        error_message = "Error while opening a semaphor\n";
        return false;
    }

    close(fd);
    return true;
}

void *SimpleIPC::data() {
    return addr;
}

void SimpleIPC::detach() {
    sem_close(sem);
    munmap(addr, mapping_size);
    sem_unlink(key.c_str());
    shm_unlink(key.c_str());
}

void SimpleIPC::lock() {
    sem_wait(sem);
}

void SimpleIPC::unlock() {
    sem_post(sem);
}

SimpleIPC::~SimpleIPC() {
    detach();
}
