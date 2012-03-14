/* 
    Conductor: High Degree of Freedom Robot Controller Framework
    Copyright (C) 2010, 2011 Robert Sherbert
    bob.sherbert@gmail.com

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

    If you are interested in licensing this software for commercial purposes
    please contact the author. The software can be licensed to you under
    non-free license terms by the copyright holder.

    As a courtesy to the author, and in the spirit of fair attribution, you are
    asked to cite the following paper if any work based upon or utilizing this
    framework is published in the scientific literature: 
    Sherbert, Robert M. and Oh, Paul Y. "Conductor: A Controller Development
    Framework for High Degree of Freedom Systems." Intelligent Robots and
    Systems (IROS), 2011 IEEE/RSJ International Conference on. 
*/

#ifndef ACES_WORD_HPP
#define ACES_WORD_HPP

#include <rtt/Logger.hpp>
#include "credentials.hpp"

namespace ACES{
    enum MODES { REFRESH=1, SET};

    class ProtoWord {
        public:
            ProtoWord();
            ProtoWord(const ProtoWord& w);
            ProtoWord(int nID=0, int dID=0, int m=0, Credentials* c=NULL);

            int getNodeID() const;
            int getDevID() const;
            int getMode() const;
            Credentials* getCred() const;
            bool setCred(Credentials* c);
        protected:
            int nodeID; //!Identify the type of node on the Device
            int devID;
            Credentials* cred;
            int mode; //!The objective of this Goal packet (Refresh, Set, etc)
    };

    template <class T>
    class Word : public ProtoWord {
        public:
            Word();
            Word(const Word& w);
            Word(T d, const Word& w);
            Word(T d, int nID=0, int dID=0, int m=0, Credentials* c=NULL);

            void printme();
            T getData() const;
            void setData(T in);
        protected:
            T data;
    };

    template <class T>
    Word<T>::Word()
     : ProtoWord(0,0,0,NULL),
       data(){
    }

    template <class T>
    Word<T>::Word(T d, int nID, int dID, int m, Credentials* c)
     : ProtoWord(nID, dID, m, c),
       data(d){
    }

    template <class T>
    Word<T>::Word( const Word &w) : ProtoWord(static_cast<const ProtoWord&>(w) ) {
        data = w.getData();
    }

    template <class T>
    Word<T>::Word(T d, const Word& w) : ProtoWord(static_cast<const ProtoWord&>(w) ){
        data = d;
    }

    template <class T>
    T Word<T>::getData() const{
        return data;
    }

    template <class T>
    void Word<T>::setData(T in){
        this->data= in;
    }

    template <class T>
    void Word<T>::printme(){
        RTT::Logger::log() << "Word ";
        if(nodeID){
            RTT::Logger::log() << "N(" << this->nodeID << "), ";
        }
        if(devID){
            RTT::Logger::log() << "D(" << this->devID << "), ";
        }
        if(mode){
            RTT::Logger::log() << "M(" << this->mode << "): ";
        }
        RTT::Logger::log() << "= " << this->data << RTT::endlog();

        if(cred){
            cred->printme();
        }
    }
}

#endif 
