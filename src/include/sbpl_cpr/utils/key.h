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
 *     * Neither the name of the University of Pennsylvania nor the names of its
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

#ifndef __KEY_H_
#define __KEY_H_

#define KEY_SIZE 2

#define INFINITECOST 1000000000

class CKey
{
    //data
public:
    long int key[KEY_SIZE];

    //constructors
public:
    CKey()
    {
#if KEY_SIZE == 1
        key[0] = 0;
#elif KEY_SIZE == 2
        key[0] = 0;
        key[1] = 0;
#else
        for(int i = 0; i < KEY_SIZE; i++)
        {
            key[i] = 0;
        }
#endif
    }

    ~CKey() { }

    //functions
public:
    void SetKeytoInfinity()
    {
        for (int i = 0; i < KEY_SIZE; i++) {
            key[i] = INFINITECOST;
        }
    }

    void SetKeytoZero()
    {
        for (int i = 0; i < KEY_SIZE; i++) {
            key[i] = 0;
        }
    }

    void operator =(CKey RHSKey)
    {
        //iterate through the keys
        //the 0ht is the most important key
#if KEY_SIZE == 1
        key[0] = RHSKey.key[0];
#elif KEY_SIZE == 2
        key[0] = RHSKey.key[0];
        key[1] = RHSKey.key[1];
#else
        for(int i = 0; i < KEY_SIZE; i++)
        key[i] = RHSKey.key[i];
#endif
    }

    CKey operator -(const CKey& RHSKey) const
    {
        CKey RetKey;

        //iterate through the keys
        //the 0ht is the most important key
        for (int i = 0; i < KEY_SIZE; i++)
            RetKey.key[i] = key[i] - RHSKey.key[i];

        return RetKey;
    }

    bool operator >(CKey& RHSKey)
    {
        //iterate through the keys
        //the 0ht is the most important key
#if KEY_SIZE == 1
        return (key[0] > RHSKey.key[0]);
#elif KEY_SIZE == 2
        return (key[0] > RHSKey.key[0] || (key[0] == RHSKey.key[0] && key[1] > RHSKey.key[1]));
#else
        for(int i = 0; i < KEY_SIZE; i++)
        {
            //compare the current key
            if(key[i] > RHSKey.key[i])
            return true;
            else if(key[i] < RHSKey.key[i])
            return false;
        }
        //all are equal
        return false;
#endif
    }

    bool operator ==(CKey& RHSKey)
    {
        //iterate through the keys
        //the 0ht is the most important key
#if KEY_SIZE == 1
        return (key[0] == RHSKey.key[0]);
#elif KEY_SIZE == 2
        return (key[0] == RHSKey.key[0] && key[1] == RHSKey.key[1]);
#else

        for(int i = 0; i < KEY_SIZE; i++)
        {
            //compare the current key
            if(key[i] != RHSKey.key[i])
            return false;
        }

        //all are equal
        return true;
#endif
    }

    bool operator !=(CKey& RHSKey)
    {
        return !(*this == RHSKey);
    }

    bool operator <(CKey& RHSKey)
    {
        //iterate through the keys
        //the 0ht is the most important key
#if KEY_SIZE == 1
        return (key[0] < RHSKey.key[0]);
#elif KEY_SIZE == 2
        return (key[0] < RHSKey.key[0] || (key[0] == RHSKey.key[0] && key[1] < RHSKey.key[1]));
#else
        for(int i = 0; i < KEY_SIZE; i++)
        {
            //compare the current key
            if(key[i] < RHSKey.key[i])
            return true;
            else if(key[i] > RHSKey.key[i])
            return false;
        }
        //all are equal
        return false;
#endif
    }

    bool operator >=(CKey& RHSKey) { return !(*this < RHSKey); }

    bool operator <=(CKey& RHSKey) { return !(*this > RHSKey); }

    long int operator [](int i) { return key[i]; }
};

#endif

