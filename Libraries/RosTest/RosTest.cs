using System.Threading;
using System;
using RosSharp.RosBridgeClient;
using std_msgs = RosSharp.RosBridgeClient.MessageTypes.Std;
//using std_srvs = RosSharp.RosBridgeClient.MessageTypes.Std;

// on ROS system:
// launch before starting:
// roslaunch rosbridge_server rosbridge_websocket.launch
// rostopic echo /publication_test

namespace RosSharp
{
    class RosTest
    {
        private RosSocket rosSocket;
        private string Uri = "ws://127.0.0.1:9090";
        private string rosId;

        public void Setup()
        {
            Console.WriteLine("__Setup");
            // Create a RosSocket with the WebSocketNetProtocol
            // Work also with default Serializer
            // Need a protocol to attach Event
            RosBridgeClient.Protocols.IProtocol protocol = new RosBridgeClient.Protocols.WebSocketNetProtocol(Uri);
            rosSocket = new RosSocket(protocol, RosSocket.SerializerEnum.Newtonsoft_JSON );
            protocol.OnConnected += OnConnected;
            protocol.OnReceive += OnReceived;
            protocol.OnClosed += OnClosed;
            Thread.Sleep(100);
            Console.WriteLine("  done");
        }
        public void Publicationtest( string msg )
        {
            Console.WriteLine("__PublicationTest"+msg);
            //string id = rosSocket.Advertise<std_msgs.String>("/publication_test");
            rosId = rosSocket.Advertise<std_msgs.String>("/publication_test");
            std_msgs.String message = new std_msgs.String
            {
                data = "publication test message data"
            };
            rosSocket.Publish(rosId, message);
            Thread.Sleep(100);
            Console.WriteLine("  done id="+rosId);
        }

        // ------------------------------------------------------ Event callback
        void OnConnected(object sender, EventArgs e)
        {
            Console.WriteLine( "*************************************** Event" );
            Console.WriteLine( "++ onConnected" );
            Console.WriteLine( "*********************************************" );
                        
        }
        void OnReceived(object sender, EventArgs e)
        {
            Console.WriteLine( "*************************************** Event" );
            Console.WriteLine( "++ onReceived" );
            Console.WriteLine( "*********************************************" );
                        
        }
        void OnClosed(object sender, EventArgs e)
        {
            Console.WriteLine( "*************************************** Event" );
            Console.WriteLine( "++ onClosed" );
            Console.WriteLine( "*********************************************" );
                        
        }        
        
        public void TearDown()
        {
            Console.WriteLine("__TearDown");
            rosSocket.Unadvertise( rosId);
            Thread.Sleep(100);

            rosSocket.Close();
            Console.WriteLine("  done");
        }
        public static void Main(string[] args)
        {
            Console.WriteLine("***** RosTest *****");
            RosTest app = new RosTest();
            app.Setup();
            for (int i = 0; i < 20; ++i)
            {
                app.Publicationtest( String.Format( " try_{0:D2}", i));
                Thread.Sleep(1000);
            }
            app.TearDown();
            Thread.Sleep(1000);
        }
    }
}
