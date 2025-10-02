#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

#include <Arduino.h>

template <typename T>
class CircularBuffer {
    public:
        CircularBuffer(int size);
        ~CircularBuffer();
        int push(T item);
        int pop(T *item);
        int pop(void);
        int peek(T *item);
        int peek(T *item, size_t index);
        int get_size();
        unsigned int get_count();
        int get_free();
        void clear();
    private:
        T *buffer;
        int size;
        int head;
        int tail;
        int count;
};

template <typename T>
CircularBuffer<T>::CircularBuffer(int size){
    this->size = size;
    this->buffer = (T *)malloc(sizeof(T) * size);
    this->head = 0;
    this->tail = 0;
    this->count = 0;
}

template <typename T>
CircularBuffer<T>::~CircularBuffer(){
    free(this->buffer);
}

template <typename T>
int CircularBuffer<T>::push(T item){
    if (this->count == this->size){
        return -1;
    }
    this->buffer[this->head] = item;
    this->head = (this->head + 1) % this->size;
    this->count++;
    return 0;
}

template <typename T>
int CircularBuffer<T>::pop(T *item){
    if (this->count == 0){
        return -1;
    }
    *item = this->buffer[this->tail];
    this->tail = (this->tail + 1) % this->size;
    this->count--;
    return 0;
}

template <typename T>
int CircularBuffer<T>::pop(void){
    if (this->count == 0){
        return -1;
    }
    this->tail = (this->tail + 1) % this->size;
    this->count--;
    return 0;
}

template <typename T>
int CircularBuffer<T>::peek(T *item){
    if (this->count == 0){
        return -1;
    }
    *item = this->buffer[this->tail];
    return 0;
}

template <typename T>
int CircularBuffer<T>::peek(T *item, size_t index){
    if (this->count == 0){
        return -1;
    }
    *item = this->buffer[(this->tail + index) % this->size];
    return 0;
}

template <typename T>
int CircularBuffer<T>::get_size(){
    return this->size;
}

template <typename T>
unsigned int CircularBuffer<T>::get_count(){
    return this->count;
}

template <typename T>
int CircularBuffer<T>::get_free(){
    return this->size - this->count;
}

template <typename T>
void CircularBuffer<T>::clear(){
    this->head = 0;
    this->tail = 0;
    this->count = 0;
}

#endif // CIRCULAR_BUFFER_H
