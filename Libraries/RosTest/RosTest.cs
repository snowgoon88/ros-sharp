using System.Threading;
using System;
using RosSharp.RosBridgeClient;
using std_msgs = RosSharp.RosBridgeClient.MessageTypes.Std;
//using std_srvs = RosSharp.RosBridgeClient.MessageTypes.Std;

// on ROS system:
// launch before starting:
// roslaunch rosbridge_server rosbridge_websocket.launch
// rostopic echo /pub_test
// rostopic pub /sub_test std_msgs/String "subscription test message data"
namespace RosSharp
{
    class RosTest
    {
        private RosSocket rosSocket;
        private string Uri = "ws://127.0.0.1:9090";
        private string pubId;
        private string subId;
        private RosBridgeClient.Protocols.IProtocol protocol;
        public void Setup()
        {
            Console.WriteLine("__Setup");
            // Create a RosSocket with the WebSocketNetProtocol
            // Work also with default Serializer
            // Need a protocol to attach Event
            protocol = new RosBridgeClient.Protocols.WebSocketNetProtocol(Uri);
            rosSocket = new RosSocket(protocol, RosSocket.SerializerEnum.Newtonsoft_JSON );
            protocol.OnConnected += OnConnected;
            protocol.OnReceive += OnReceived;
            protocol.OnClosed += OnClosed;
            Thread.Sleep(100);
            Console.WriteLine("  done");
        }

        public void CloseConnection()
        {
            Console.WriteLine("  Closing");
            rosSocket.Close();
        }
        public void Reconnect()
        {
            Console.WriteLine("  Reconnecting");
            protocol.Connect();
        }


        public void Publicationtest( string msg )
        {
            Console.WriteLine("__PublicationTest"+msg);
            IsAlive();
            //string id = rosSocket.Advertise<std_msgs.String>("/pub_test");
            pubId = rosSocket.Advertise<std_msgs.String>("/pub_test");
            std_msgs.String message = new std_msgs.String
            {
                data = "publication test message data" + msg
            };

            rosSocket.Publish(pubId, message);
            Thread.Sleep(100);
            Console.WriteLine("  done id="+pubId);
        }
        public void SubscriptionTest( string msg )
        {
            Console.WriteLine("__SubscriptionTest" + msg);
            IsAlive();
            subId = rosSocket.Subscribe<std_msgs.String>("/sub_test", SubscriptionHandler);

        }
        private void SubscriptionHandler(std_msgs.String message)
        {
            Console.WriteLine("Received: " + message.data);
        }

        // ------------------------------------------------------------- IsAlive
        bool IsAlive()
        {
            if( protocol.IsAlive() )
            {
                Console.WriteLine("++ still Alive");
                return true;
            }
            else
            {
               Console.WriteLine("-- NOT Alive");
                return false; 
            }
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
            rosSocket.Unadvertise( pubId);
            rosSocket.Unsubscribe( subId);
            Thread.Sleep(100);

            rosSocket.Close();
            Console.WriteLine("  done");
        }
        public static void Main(string[] args)
        {
            Console.WriteLine("***** RosTest *****");
            RosTest app = new RosTest();
            app.Setup();
            app.SubscriptionTest( " listening" );
            for (int i = 0; i < 20; ++i)
            {
                app.Publicationtest( String.Format( " try_{0:D2}", i));
                Thread.Sleep(1000);
            }

            app.CloseConnection();
            Thread.Sleep(3000);
            app.Setup();
            app.SubscriptionTest( " listening again" );
            Thread.Sleep(1000);
            for (int i = 0; i < 20; ++i)
            {
                app.Publicationtest( String.Format( " try_{0:D2}", i));
                Thread.Sleep(1000);
            }

            app.CloseConnection();
            Thread.Sleep(1000);
            for (int i = 0; i < 20; ++i)
            {
                app.Publicationtest( String.Format( " try_{0:D2}", i));
                Thread.Sleep(1000);
            }
            
            
            app.TearDown();
            Thread.Sleep(2000);
        }
    }
}
