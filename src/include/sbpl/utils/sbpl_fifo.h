/*
 * Copyright (c) 2008, Maxim Likhachev
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Carnegie Mellon University nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef SBPL_FIFO
#define SBPL_FIFO

#include <cstdio>

template<class T>
class sbpl_fifo
{
public:
    sbpl_fifo(unsigned int size)
    {
        q_ = new T[size];
        head_ = 0;
        tail_ = 0;
        size_ = size;
    }

    ~sbpl_fifo()
    {
        delete[] q_;
    }

    bool insert(T val)
    {
        int t_val = tail_;
        if (t_val == head_ + 1 || (t_val == 0 && head_ + 1 == size_)) {
            printf("ERROR: Trying to insert when FIFO is full!\n");
            return false;
        }
        q_[head_] = val;
        head_++;
        if (head_ == size_) head_ = 0;
        return true;
    }

    bool remove(T* val)
    {
        if (head_ == tail_) {
            printf("ERROR: Trying to remove when FIFO is empty!\n");
            return false;
        }
        *val = q_[tail_];
        tail_++;
        if (tail_ == size_) tail_ = 0;
        return true;
    }

    bool empty()
    {
        return head_ == tail_;
    }

    void clear()
    {
        head_ = 0;
        tail_ = 0;
    }

private:
    int head_;
    int tail_;
    int size_;
    T* q_;
};

#endif
