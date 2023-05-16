#ifndef EVENTQUEUE_H
#define EVENTQUEUE_H
/*
MIT License
Copyright (c) 2018 Jukka-Pekka Sarjanen
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#include <inttypes.h>

#ifdef ARDUINO
#include <SimplyAtomic.h>
#else
#define ATOMIC(x)
#warning "Not thread safe in host env!"
#endif

///Array based thread safe queue for Arduino.
/// Implement FIFO type queue that use array as ringbuffer
/// to store messages.
///
template <class T,uint8_t s=8>
class EventQueue{

public:
    EventQueue():_in(0),_out(0),_count(0),_s(s){;}
    ///Pop first item from queue.
    /// \return status of the operation.
    /// \retval true when successfully pop item from queue.
    /// \retval false in case empty queue.
    ///
    bool getQ(T &e ///< Item to be pop out from queue.
              ){
        bool rc=false;
        ATOMIC() {
            if( _count > 0){
                _count--;
                e = _queue[_out%_s];
                _out = (++_out/_s)?0:_out;
                rc = true;
            }
        }
        return rc;

    }

    /// Insert item to queue.
    /// \return Status of the operation.
    /// \retval true in case sucess.
    /// \retval false in case queue is full and item can not be added.
    ///
    bool putQ(const T &e ///< Reference item to be put in queue.
              ){
        bool rc=false;
        ATOMIC(){
            if( _count <_s){
                _count++;
                _queue[_in%_s] = e;
                _in =  (++_in/_s)?0:_in;
                rc = true;
            }
        }
        return rc;
    }

private:
    uint8_t _in;
    uint8_t _out;
    uint8_t _count;
    const    uint8_t _s;
    T       _queue[s];
};

#endif // EVENTQUEUE
