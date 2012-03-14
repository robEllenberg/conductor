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

namespace ACES{
    template <class PD, class HW>    
    Protocol<PD,HW>::Protocol(std::string cfg, std::string args) :
      ProtoProtocol(cfg,args) 
    {
        this->ports()->addEventPort("RxDS", rxDownStream).doc(
                               "DownStream (from Device) Reception");
        this->ports()->addEventPort("RxUS", rxUpStream).doc(
                               "UpStream (from Hardware) Reception");
        this->ports()->addPort("TxDS", txDownStream).doc(
                               "DownStream (to Hardware) Transmission");
        this->ports()->addPort("TxUS", txUpStream).doc(
                               "UpStream (to Device) Transmission");
    }

    template <class PD, class HW>    
    void Protocol<PD,HW>::updateHook(){
        Word<HW> usIn;
        Word<PD> usOut;
        Word<PD> dsIn;
        Message<HW> dsOut;

        if(rxUpStream.read(usIn) == RTT::NewData){
            do{
                if( (processUS(usIn, usOut)) ){
                    RTT::Logger::log(RTT::Logger::Debug) 
                                       << "(Protocol: "
                                       << name << ") got US" << RTT::endlog();
                    txUpStream.write(usOut);
                }
            }while( rxUpStream.read(usIn) == RTT::NewData );
        }else{
            while( rxDownStream.read(dsIn) == RTT::NewData ){
                if( (processDS(dsIn, dsOut)) ){
                    RTT::Logger::log(RTT::Logger::Debug) 
                                       << "(Protocol: " << name << ") got DS"
                                       << RTT::endlog();
                    txDownStream.write(dsOut);
                }
            }
            txDSPending();
        }

    }

    template <class PD, class HW>    
    bool Protocol<PD, HW>::processDS(Word<PD>& dsIn, Message<HW>& dsOut){
        //TODO - make this typecast and pass data
        //dsOut.push(static_cast<Word<PD> >(dsIn));
        return true;
    }

    template <class PD, class HW>    
    bool Protocol<PD, HW>::processUS(Word<HW>& usIn, Word<PD>& usOut){

        //In virtual functions, specialized processing function from
        //PDord->HWWord If we're going to implement some kind of state machine
        //or whatever it needs to be done here. This default is only
        //meaningful for type-same protocols & hardware (not many at all)
        //usOut = static_cast<Word<HW> >(usIn);

        return true;
    }

    template <class PD, class HW>    
    void Protocol<PD, HW>::txDSPending(){
    }
}
