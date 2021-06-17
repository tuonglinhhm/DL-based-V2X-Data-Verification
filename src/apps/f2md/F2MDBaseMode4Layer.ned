//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
//


package lte.apps.f2md;
import lte.apps.mode4App.Mode4BaseApp;

simple F2MDBaseMode4Layer like Mode4BaseApp
{
    parameters:
    	bool sendAlerts = default(true); //tell the applayer to periodically send beacons
        int packetSize = default(10); // Size of the actual packet itself
        int priority = default(3); // Priority of the packets sent
        int duration = default(1000); // MS before packet must be dropped
        double period @unit("s") = default(0.1s);
        int macNodeId = default(0);
        
        int headerLength = default(88bit) @unit(bit); //header length of the application
        bool sendBeacons = default(true); //tell the applayer to periodically send beacons
        int beaconLengthBits = default(256bit) @unit(bit); //the length of a beacon packet
        int beaconUserPriority = default(7); //the user priority (UP) of the beacon messages
        double beaconInterval = default(1s) @unit(s); //the intervall between 2 beacon messages
        
        @signal[sentMsg];
        @statistic[sentMsg](title="Messages sent"; unit=""; source="sentMsg"; record=sum,vector);
        @signal[delay];
        @statistic[delay](title="Message Delay"; unit="s"; source="delay"; record=mean,vector);    
        @signal[rcvdMsg]; 
        @statistic[rcvdMsg](title="Messages Received"; unit="s"; source="rcvdMsg"; record=sum,vector);
        @signal[cbr];
        @statistic[cbr](title="Channel Busy Ratio"; unit=""; source="cbr"; record=vector);         
        @display("i=block/source");
    gates:
        input lowerGateIn; // FROM NIC
        output lowerGateOut;  // TO NIC
}
