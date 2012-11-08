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

#ifndef __HEAP_H_
#define __HEAP_H_

#include <sbpl/planners/planner.h>
#include <sbpl/utils/key.h>

//the maximum size of the heap
#define HEAPSIZE 20000000 
#define HEAPSIZE_INIT 5000

struct HEAPELEMENT
{
    AbstractSearchState *heapstate;
    CKey key;
};

typedef struct HEAPELEMENT heapelement;

class CHeap
{
    //data
public:
    int percolates; //for counting purposes
    heapelement* heap;
    int currentsize;
    int allocated;

    //constructors
public:
    CHeap();
    ~CHeap();

    //functions
public:
    bool emptyheap();
    bool fullheap();
    bool inheap(AbstractSearchState *AbstractSearchState);
    CKey getkeyheap(AbstractSearchState *AbstractSearchState);
    void makeemptyheap();
    void insertheap(AbstractSearchState *AbstractSearchState, CKey key);
    void deleteheap(AbstractSearchState *AbstractSearchState);
    void updateheap(AbstractSearchState *AbstractSearchState, CKey NewKey);
    AbstractSearchState *getminheap();
    AbstractSearchState *getminheap(CKey& ReturnKey);
    CKey getminkeyheap();
    AbstractSearchState *deleteminheap();
    void makeheap();
    void insert_unsafe(AbstractSearchState* state, CKey key);
    void updateheap_unsafe(AbstractSearchState* AbstractSearchState, CKey NewKey);
    void deleteheap_unsafe(AbstractSearchState* AbstractSearchState);

private:
    void percolatedown(int hole, heapelement tmp);
    void percolateup(int hole, heapelement tmp);
    void percolateupordown(int hole, heapelement tmp);

    void growheap();
    void sizecheck();
};

struct HEAPINTELEMENT
{
    AbstractSearchState *heapstate;
    int key;
};

typedef struct HEAPINTELEMENT heapintelement;

class CIntHeap
{
    //data
public:
    int percolates; //for counting purposes
    heapintelement* heap;
    int currentsize;
    int allocated;

    //constructors
public:
    CIntHeap();
    CIntHeap(int initial_size);
    ~CIntHeap();

    //functions
public:
    bool emptyheap();
    bool fullheap();
    bool inheap(AbstractSearchState *AbstractSearchState);
    int getkeyheap(AbstractSearchState *AbstractSearchState);
    void makeemptyheap();
    void insertheap(AbstractSearchState *AbstractSearchState, int key);
    void deleteheap(AbstractSearchState *AbstractSearchState);
    void updateheap(AbstractSearchState *AbstractSearchState, int NewKey);
    AbstractSearchState *getminheap();
    AbstractSearchState *getminheap(int& ReturnKey);
    int getminkeyheap();
    AbstractSearchState *deleteminheap();
    void makeheap();

private:
    void percolatedown(int hole, heapintelement tmp);
    void percolateup(int hole, heapintelement tmp);
    void percolateupordown(int hole, heapintelement tmp);

    void growheap();
    void sizecheck();
};

#endif

