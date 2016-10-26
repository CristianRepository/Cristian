using System;
using System.Collections;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.IO.Ports;
using GraphLib;
using ChartDirector;
using System.Threading;
using System.Runtime.InteropServices;
using System.Security.Cryptography;
using System.Windows.Shapes;



namespace Piezo_Interface
{
    public partial class Form1 : Form
    {
        //ChartDirector.WinChartViewer chartViewer;
        public delegate void AddListItem();
        public AddListItem myDelegate;
        PrecisionTimer.Timer myTimer = null;
        int point = 0;
        double dataA;
        double dataB;
        double dataA_old;
        double dataB_old;

        double X_acc;
        double X_acc_old;
        double Y_acc;
        double Y_acc_old;
        double Z_acc;
        double Z_acc_old;


        byte alarm_detected = 0;
        UInt16 Old_threshold = 0;
        bool threshold_cmd_sem = true;
        byte Electronic_Gain = 1;
        UInt16 nSampleValue = 0;
        UInt16 soglia_intervento = 0;
        SerialPort sp;
        private packet pkt;
        byte[] buffer;
        int buffer_temp_index;
        bool Serial_port_open = false;
        int packet_length = 0;
        bool Start_acquisition = false;
        UInt32 COUNTDOWN_VALUE = 2;
        UInt32 Countdown;

        bool parameter_saved = false;


        public const int PL_LENGTH = 8;

        static class Command
        {
            public const char STX_CONST = '#';
            public const char ETX_CONST = '@';
            public const char CMD_WRITE = 'W';
            public const char CMD_READ = 'R';

        }
        static class Sub_Command
        {
            public const char PING = 'A';
            public const char READ_SENSOR = 'B';
            public const char SET_NCAMP = 'C';
            public const char SET_THSD = 'D';
            public const char SET_GAIN_1 = 'E';
            public const char GET_NCAMP = 'F';
            public const char GET_THSD = 'G';
            public const char GET_GAIN = 'H';
            public const char SAVE_PAR = 'I';
            public const char SET_SOGLIA = 'L';
            public const char GET_SOGLIA = 'M';
            public const char SET_GAIN_2 = 'N';
        }



        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct packet
        {
            public char STX;
            public char cmd;
            public char sub_cmd;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = PL_LENGTH)]
            public byte[] payload;
            public char ETX;
        }


        static class opamp_pot_mux
        {
            /** Gain = R2/R1 = 1/7 */
            public const int OPAMP_POT_MUX_14R_2R = 1;
            /** Gain = R2/R1 = 1/3 */
            public const int OPAMP_POT_MUX_12R_4R = 2;
            /** Gain = R2/R1 = 1 */
            public const int OPAMP_POT_MUX_8R_8R = 3;
            /** Gain = R2/R1 = 1 + 2/3 */
            public const int OPAMP_POT_MUX_6R_10R = 4;
            /** Gain = R2/R1 = 3 */
            public const int OPAMP_POT_MUX_4R_12R = 5;
            /** Gain = R2/R1 = 4 + 1/3 */
            public const int OPAMP_POT_MUX_3R_13R = 6;
            /** Gain = R2/R1 = 7 */
            public const int OPAMP_POT_MUX_2R_14R = 7;
            /** Gain = R2/R1 = 15 */
            public const int OPAMP_POT_MUX_R_15R = 8;
        };

        
        private const int sampleSize = 128;
        private const int SampleNumber = 128 * 20;
        private Queue<int> SampleNumberQueue = new Queue<int>();

        //private DateTime[] timeStamps = new DateTime[SampleNumber];
        //private double[] dataSeriesA = new double[SampleNumber];

        private DateTime[] timeStamps = new DateTime[sampleSize * 2];
        private double[] dataSeriesA = new double[sampleSize * 2];
        private double[] dataSeriesB = new double[sampleSize * 2];
        //private double[] dataSeriesC = new double[sampleSize];


        private DateTime nextDataTime = DateTime.Now;






        public Form1()
        {
            //myTimer = new PrecisionTimer.Timer();
            //myTimer.period

            //chartViewer = new ChartDirector.WinChartViewer();
            InitializeComponent();

            var ports = SerialPort.GetPortNames();
            SerialPort_List.Items.AddRange(ports);
            SerialPort_List.DataSource = ports;

            threshold_value.Text = ((UInt16)(Threshold_Limit.Maximum - Threshold_Limit.Value)).ToString();


            myDelegate = new AddListItem(AddListItemMethod);

            //display.Smoothing = System.Drawing.Drawing2D.SmoothingMode.None;

            this.Load += Form1_Load;

            // Data generation rate
            dataRateTimer.Interval = 250;
            Countdown = COUNTDOWN_VALUE;

            // Initialize data buffer to no data.
            for (int i = 0; i < timeStamps.Length; ++i)
                timeStamps[i] = DateTime.MinValue;

            // Now can start the timers for data collection and chart update
            dataRateTimer.Start();

        }

        static byte[] getBytes(object str)
        {
            int size = Marshal.SizeOf(str);
            byte[] arr = new byte[size];
            IntPtr ptr = Marshal.AllocHGlobal(size);

            Marshal.StructureToPtr(str, ptr, true);
            Marshal.Copy(ptr, arr, 0, size);
            Marshal.FreeHGlobal(ptr);

            return arr;
        }

        static T fromBytes<T>(byte[] arr)
        {
            T str = default(T);

            int size = Marshal.SizeOf(str);
            IntPtr ptr = Marshal.AllocHGlobal(size);

            Marshal.Copy(arr, 0, ptr, size);

            str = (T)Marshal.PtrToStructure(ptr, str.GetType());
            Marshal.FreeHGlobal(ptr);

            return str;
        }

        public void AddListItemMethod()
        {
            //Console.WriteLine(dataA);
            drawChart(winChartViewer1);
            //winChartViewer1.updateViewPort(true, false);
        }

        void Form1_Load(object sender, EventArgs e)
        {
            
            




        }
        private void button1_Click(object sender, EventArgs e)
        {
            string portName = SerialPort_List.SelectedItem.ToString();
            serialPort1 = new SerialPort(portName);

            if (button1.Text == "OPEN")
            {
                button1.Text = "OPENED";
                button1.Enabled = false;

                START.Visible = true;
                Threshold_Limit.Enabled = true;
                Set_nSample.Enabled = true;
                Gain_selector.Enabled = true;

                // var serialPort1 = new SerialPort("USBSER000");

                serialPort1.Parity = Parity.None;
                serialPort1.StopBits = StopBits.One;
                serialPort1.DataBits = 8;
                serialPort1.Handshake = Handshake.None;
                serialPort1.DataReceived += new System.IO.Ports.SerialDataReceivedEventHandler(DataReceivedHandler);
                serialPort1.ErrorReceived += new System.IO.Ports.SerialErrorReceivedEventHandler(ErrorReceivedHandler);



                serialPort1.BaudRate = 115200;

                //int packet_length = Marshal.SizeOf(pkt);


                try
                {
                    serialPort1.Open();
                    this.AlarmDetected.On = true;

                    {
                        pkt = new packet();
                        packet_length = Marshal.SizeOf(pkt);
                        buffer = new byte[packet_length];
                        pkt.cmd = Command.CMD_READ;
                        pkt.STX = Command.STX_CONST;
                        pkt.ETX = Command.ETX_CONST;
                        pkt.sub_cmd = Sub_Command.PING;
                        pkt.payload = new byte[PL_LENGTH];

                        serialPort1.Write(getBytes(pkt), 0, packet_length);
                    }
                }
                catch (Exception ex)
                {
                    MessageBox.Show("Error: " + ex.ToString(), "ERROR");
                }
            }
            //else if (button1.Text == "CLOSE")
            //{
            //    button1.Text = "OPEN";
                
            //    START.Visible = false;
            //    STOP.Visible = false;

            //    Serial_port_open = false;
            //    serialPort1.Close(); 
           
            //}
        }

        private Queue<byte> recievedData = new Queue<byte>();

        private void ErrorReceivedHandler(object sender, SerialErrorReceivedEventArgs e)
        {
            //Console.WriteLine("ERROR SERIAL PORT");
            MessageBox.Show("ERROR SERIAL PORT");
        }

        private void DataReceivedHandler(object sender, SerialDataReceivedEventArgs e)
        {
            sp = (SerialPort)sender;

//            if (Serial_port_open == true)
//            {
                try
                {

                    int nbyte = sp.BytesToRead;
                    //Console.WriteLine("RX");
                    //Console.WriteLine(nbyte);


                    if (nbyte > 0)
                    {
                        while (nbyte > 0)
                        {
                            char indata = (char)sp.ReadByte();

                            switch (indata)
                            {
                                case Command.STX_CONST:
                                    buffer_temp_index = 0;
                                    buffer[buffer_temp_index] = (byte)indata;
                                    buffer_temp_index++;

                                    break;
                                case Command.ETX_CONST:

                                    buffer[buffer_temp_index] = (byte)indata;

                                    packet ack_pkt = fromBytes<packet>(buffer);
                                    //char cmd_temp = ack_pkt.cmd;
                                    //Console.WriteLine(cmd_temp);


                                    if ((ack_pkt.STX == Command.STX_CONST) && (ack_pkt.ETX == Command.ETX_CONST))
                                    {
                                        //Console.WriteLine("Paket RECEIVED\n\r");
                                        switch (ack_pkt.cmd)
                                        {
                                            case Command.CMD_READ:
                                                switch (ack_pkt.sub_cmd)
                                                {
                                                    case Sub_Command.READ_SENSOR:
                                                        //dataA = 4096;
                                                        //dataA -= (double)(ack_pkt.payload[1] << 8 | ack_pkt.payload[0]);
                                                        dataA = (double)(ack_pkt.payload[1] << 8 | ack_pkt.payload[0]);
                                                        dataB = (double)(ack_pkt.payload[3] << 8 | ack_pkt.payload[2]);
                                                        alarm_detected = ack_pkt.payload[4];



                                                    //Console.WriteLine(dataA);

                                                    //if ((dataA == 0)||(dataA < 0)||(dataA > 0x0FFF))
                                                    //    dataA = dataA_old;
                                                    //if ((dataB == 0) ||(dataB < 0)||(dataB > 0x0FFF))
                                                    //    dataB = dataB_old;

                                                        
                                                   
                                                    shiftData(dataSeriesA, dataA);
                                                    shiftData(dataSeriesB, dataB);
                                                    this.Invoke(this.myDelegate);

                                                    X_acc = ack_pkt.payload[5];
                                                    Y_acc = ack_pkt.payload[6];
                                                    Z_acc = ack_pkt.payload[7];

                                                    BeginInvoke(new Action(() => XAcceleration_TextBox.Text = X_acc.ToString()));
                                                    BeginInvoke(new Action(() => XAcceleration_TextBox.Update()));
                                                    BeginInvoke(new Action(() => YAcceleration_TextBox.Text = Y_acc.ToString()));
                                                    BeginInvoke(new Action(() => YAcceleration_TextBox.Update()));
                                                    BeginInvoke(new Action(() => ZAcceleration_TextBox.Text = Z_acc.ToString()));
                                                    BeginInvoke(new Action(() => ZAcceleration_TextBox.Update()));

                                                    if ((Start_acquisition == true) && (nbyte < 10))
                                                    {
                                                        sp.Write(getBytes(pkt), 0, packet_length);
                                                        Countdown = COUNTDOWN_VALUE;
                                                    }

                                                    dataA_old = dataA;
                                                    dataB_old = dataB;
                                                    X_acc_old = X_acc;
                                                    Y_acc_old = Y_acc;
                                                    Z_acc_old = Z_acc;

                                                        break;
                                                    case Sub_Command.PING:
                                                        Console.WriteLine(ack_pkt.payload[1] << 8 | ack_pkt.payload[0]);
                                                        Serial_port_open = true;

                                                        //After recognized the board
                                                        //*************** Set the threshold *********************
                                                        UInt16 thresholdValue_temp = Convert.ToUInt16(this.Threshold_Limit.Maximum);
                                                        if (serialPort1.IsOpen == true)
                                                        {
                                                            if (Start_acquisition == false)
                                                            {
                                                                pkt = new packet();
                                                                packet_length = Marshal.SizeOf(pkt);
                                                                buffer = new byte[packet_length];
                                                                pkt.cmd = Command.CMD_READ;
                                                                pkt.STX = Command.STX_CONST;
                                                                pkt.ETX = Command.ETX_CONST;
                                                                pkt.sub_cmd = Sub_Command.GET_THSD;

                                                                sp.Write(getBytes(pkt), 0, packet_length);
                                                            }
                                                        }
                                                        //*************** Set the Gain *********************
                                                        //if (serialPort1.IsOpen == true)
                                                        //{
                                                        //    if (Start_acquisition == false)
                                                        //    {
                                                        //        pkt = new packet();
                                                        //        packet_length = Marshal.SizeOf(pkt);
                                                        //        buffer = new byte[packet_length];
                                                        //        pkt.cmd = Command.CMD_READ;
                                                        //        pkt.STX = Command.STX_CONST;
                                                        //        pkt.ETX = Command.ETX_CONST;
                                                        //        pkt.sub_cmd = Sub_Command.GET_GAIN;

                                                        //        sp.Write(getBytes(pkt), 0, packet_length);
                                                        //    }
                                                        //}
                                                    //*************** Set the n camp *********************
                                                    if (serialPort1.IsOpen == true)
                                                    {
                                                        if (Start_acquisition == false)
                                                        {
                                                            pkt = new packet();
                                                            packet_length = Marshal.SizeOf(pkt);
                                                            buffer = new byte[packet_length];
                                                            pkt.cmd = Command.CMD_READ;
                                                            pkt.STX = Command.STX_CONST;
                                                            pkt.ETX = Command.ETX_CONST;
                                                            pkt.sub_cmd = Sub_Command.GET_NCAMP;

                                                            sp.Write(getBytes(pkt), 0, packet_length);
                                                        }
                                                    }
                                                    //*************** Set the soglia camp *********************
                                                    if (serialPort1.IsOpen == true)
                                                    {
                                                        if (Start_acquisition == false)
                                                        {
                                                            pkt = new packet();
                                                            packet_length = Marshal.SizeOf(pkt);
                                                            buffer = new byte[packet_length];
                                                            pkt.cmd = Command.CMD_READ;
                                                            pkt.STX = Command.STX_CONST;
                                                            pkt.ETX = Command.ETX_CONST;
                                                            pkt.sub_cmd = Sub_Command.GET_SOGLIA;

                                                            sp.Write(getBytes(pkt), 0, packet_length);
                                                        }
                                                    }
                                                    break;
                                                case Sub_Command.GET_NCAMP:
                                                    nSampleValue = (UInt16)(ack_pkt.payload[1] << 8 | ack_pkt.payload[0]);
                                                    BeginInvoke(new Action(() => nSample_Value.Text = nSampleValue.ToString()));
                                                    BeginInvoke(new Action(() => nSample_Value.Update()));
                                                    break;
                                                case Sub_Command.GET_THSD:
                                                    Old_threshold = (UInt16)(ack_pkt.payload[1] << 8 | ack_pkt.payload[0]);
                                                    BeginInvoke(new Action(() => threshold_value.Text = Old_threshold.ToString()));
                                                    BeginInvoke(new Action(() => threshold_value.Update()));
                                                    BeginInvoke(new Action(() => Threshold_Limit.Value = Old_threshold));
                                                    break;
                                                case Sub_Command.GET_GAIN:
                                                    Electronic_Gain = ack_pkt.payload[0];
                                                    int Gain_index_tmp = Convert.ToInt32(Electronic_Gain);
                                                    Gain_selector.SelectedIndex = Gain_index_tmp - 1;
                                                    break;
                                                case Sub_Command.GET_SOGLIA:
                                                    soglia_intervento = (UInt16)(ack_pkt.payload[1] << 8 | ack_pkt.payload[0]);
                                                    BeginInvoke(new Action(() => textBox1.Text = soglia_intervento.ToString()));
                                                    BeginInvoke(new Action(() => textBox1.Update()));
                                                    break;
                                                default:
                                                        break;
                                                }
                                                break;
                                            case Command.CMD_WRITE:
                                                switch (ack_pkt.sub_cmd)
                                                {
                                                    case Sub_Command.SET_NCAMP:
                                                        nSampleValue = (UInt16)((ack_pkt.payload[1] << 8) | (ack_pkt.payload[0]));
                                                        this.nSample_Value.Text = nSampleValue.ToString();
                                                        break;
                                                    case Sub_Command.SET_THSD:
                                                        Old_threshold = (UInt16)((ack_pkt.payload[0]) | (ack_pkt.payload[1] << 8));
                                                        threshold_cmd_sem = true;
                                                        break;
                                                    case Sub_Command.SET_GAIN_1:
                                                        Electronic_Gain = ack_pkt.payload[0];
                                                        break;
                                                    case Sub_Command.SAVE_PAR:
                                                        parameter_saved = true;
                                                        break;
                                                    case Sub_Command.SET_SOGLIA:
                                                        soglia_intervento = (UInt16)((ack_pkt.payload[0]) | (ack_pkt.payload[1] << 8));
                                                    break;
                                                default:
                                                        break;
                                                }
                                                break;
                                            default:
                                                break;
                                        }
                                    }
                                    else
                                    {
                                        //Console.WriteLine("Paket error -> Not recognized\n\r");
                                        //MessageBox.Show("Paket error -> Not recognized\n\r");
                                        sp.Close();
                                        sp.Open();
                                        if (Start_acquisition == true)
                                        {
                                            sp.Write(getBytes(pkt), 0, packet_length);
                                            Thread.Sleep(500);
                                        }
                                    }
                                    break;
                                default:
                                    if (buffer_temp_index < packet_length)
                                    {
                                        buffer[buffer_temp_index] = (byte)indata;
                                        buffer_temp_index++;
                                    }
                                    else
                                    {
                                        //Console.WriteLine("buffer_temp_index exceed the index of array\n\r");
                                        //MessageBox.Show("buffer_temp_index exceed the index of array\n\r");
                                        buffer_temp_index = 0;
                                    }
                                    break;
                            }

                            nbyte--;
                        }
                    }
                    else
                    {
                        sp.Write(getBytes(pkt), 0, packet_length);
                        Console.WriteLine("n_byte = 0 -> send token");
                        //MessageBox.Show("n_byte = 0 -> send token");
                    }

                }
                catch (Exception ioe)
                {
                    //Console.WriteLine(ioe);
                    MessageBox.Show(ioe.ToString());
                    sp.Dispose();      // This will cause another IOException
                }
            //}
            
           // else
            //{
                //sp.Close();
           // }
}

        void processData()
        {
            // Determine if we have a "packet" in the queue
            if (recievedData.Count > sampleSize * 2 - 1)
            {
                int index;
                int recievedData_length = recievedData.Count;
                byte[] data_acquired = new byte[recievedData_length];
                int[] data_acquired_and_process = new int[sampleSize];

                for (index = 0; index < recievedData_length; index++)
                {
                    data_acquired[index] = recievedData.Dequeue();
                }

                for (index = 0; index < sampleSize; index++)
                {
                    data_acquired_and_process[index] = ((data_acquired[index * 2]) | (data_acquired[index * 2 + 1] << 8));
                }

                data_acquired_and_process.ToList().ForEach(b => SampleNumberQueue.Enqueue(b));


                if (SampleNumberQueue.Count > SampleNumber - 1)
                {
                    int index2;
                    int recievedData_length2 = SampleNumberQueue.Count;
                    for (index2 = 0; index2 < recievedData_length2; index2++)
                    {
                        dataSeriesA[index2] = SampleNumberQueue.Dequeue();
                    }
                }

                this.Invoke(this.myDelegate);

            }
        }
        

        private void winChartViewer1_ViewPortChanged(object sender, WinViewPortEventArgs e)
        {
            try
            {
                drawChart(winChartViewer1);
            }
            catch (Exception ioe)
            {
                //Console.WriteLine(ioe);
                MessageBox.Show(ioe.ToString());
            }
        }
        private void dataRateTimer_Tick(object sender, EventArgs e)
        {
            do
            {
                //
                // In this demo, we use some formulas to generate new values. In real applications,
                // it may be replaced by some data acquisition code.
                //
                double p = nextDataTime.Ticks / 10000000.0 * 4;

                shiftData(timeStamps, nextDataTime);

                // Update nextDataTime. This is needed by our data generator. In real applications,
                // you may not need this variable or the associated do/while loop.
                nextDataTime = nextDataTime.AddMilliseconds(dataRateTimer.Interval);
            }
            while (nextDataTime < DateTime.Now);



            if (Serial_port_open == true)
            {
                if (Start_acquisition == true)
                {
                    if (Countdown == 0)
                    {
                        serialPort1.Write(getBytes(pkt), 0, packet_length);
                        Countdown = COUNTDOWN_VALUE;
                    }
                    else
                    {
                        if (Countdown > 0)
                            Countdown--;
                    }
                }
            }

            // We provide some visual feedback to the numbers generated, so you can see the
            // values being generated.
            //valueA.Text = dataSeriesA[dataSeriesA.Length - 1].ToString(".##");
            //valueB.Text = dataSeriesB[dataSeriesB.Length - 1].ToString(".##");
            //valueC.Text = dataSeriesC[dataSeriesC.Length - 1].ToString(".##");
        }

        //
        // Utility to shift a double value into an array
        //
        private void shiftData(double[] data, double newValue)
        {
            for (int i = 1; i < data.Length; ++i)
                data[i - 1] = data[i];
            data[data.Length - 1] = newValue;
        }

        //
        // Utility to shift a DataTime value into an array
        //
        private void shiftData(DateTime[] data, DateTime newValue)
        {
            for (int i = 1; i < data.Length; ++i)
                data[i - 1] = data[i];
            data[data.Length - 1] = newValue;
        }

        private void drawChart(WinChartViewer viewer)
        {
            // Create an XYChart object 600 x 270 pixels in size, with light grey (f4f4f4) 
            // background, black (000000) border, 1 pixel raised effect, and with a rounded frame.

            XYChart c = new XYChart(800, 650, 0xf4f4f4, 0x000000, 1);


            c.setRoundedFrame(Chart.CColor(BackColor));

            // Set the plotarea at (55, 62) and of size 520 x 175 pixels. Use white (ffffff) 
            // background. Enable both horizontal and vertical grids by setting their colors to 
            // grey (cccccc). Set clipping mode to clip the data lines to the plot area.
            c.setPlotArea(55, 62, 640, 500, 0xffffff, -1, -1, 0xcccccc, 0xcccccc);
            c.setClipping();

            // Add a title to the chart using 15 pts Times New Roman Bold Italic font, with a light
            // grey (dddddd) background, black (000000) border, and a glass like raised effect.
            c.addTitle("Piezo Sensors Interface Output", "Times New Roman Bold Italic", 15
                ).setBackground(0xdddddd, 0x000000, Chart.glassEffect());

            // Add a legend box at the top of the plot area with 9pts Arial Bold font. We set the 
            // legend box to the same width as the plot area and use grid layout (as opposed to 
            // flow or top/down layout). This distributes the 3 legend icons evenly on top of the 
            // plot area.
            LegendBox b = c.addLegend2(55, 33, 3, "Arial Bold", 9);
            b.setBackground(Chart.Transparent, Chart.Transparent);
            b.setWidth(520);

            // Configure the y-axis with a 10pts Arial Bold axis title
            c.yAxis().setTitle("Intensity", "Arial Bold", 10);

            // Configure the x-axis to auto-scale with at least 75 pixels between major tick and 15 
            // pixels between minor ticks. This shows more minor grid lines on the chart.
            c.xAxis().setTickDensity(75, 15);

            // Set the axes width to 2 pixels
            c.xAxis().setWidth(2);
            c.yAxis().setWidth(2);

            // Now we add the data to the chart
            DateTime lastTime = timeStamps[timeStamps.Length - 1];
            if (lastTime != DateTime.MinValue)
            {
                // Set up the x-axis scale. In this demo, we set the x-axis to show the last 240 
                // samples, with 250ms per sample.
                c.xAxis().setDateScale(lastTime.AddSeconds(
                   -dataRateTimer.Interval * timeStamps.Length / 1000), lastTime);

                // Set the x-axis label format
                c.xAxis().setLabelFormat("{value|hh:nn:ss}");

                // Create a line layer to plot the lines
                LineLayer layer = c.addLineLayer2();
                //SplineLayer layer = c.addSplineLayer();
                

                // The x-coordinates are the timeStamps.
                //layer.setXData(0, timeStamps.Length);
                layer.setXData(timeStamps);

                // The 3 data series are used to draw 3 lines. Here we put the latest data values
                // as part of the data set name, so you can see them updated in the legend box.
                layer.addDataSet(dataSeriesA, 0xff0000, "Alpha: <*bgColor=FFCCCC*>" +
                    c.formatValue(dataSeriesA[dataSeriesA.Length - 1], " {value|2} "));
                layer.addDataSet(dataSeriesB, 0x00cc00, "Beta: <*bgColor=CCFFCC*>" +
                    c.formatValue(dataSeriesB[dataSeriesB.Length - 1], " {value|2} "));
                //layer.addDataSet(dataSeriesC, 0x0000ff, "Gamma: <*bgColor=CCCCFF*>" +
                //    c.formatValue(dataSeriesC[dataSeriesC.Length - 1], " {value|2} "));

            }

            //Assign the chart to the WinChartViewer
            viewer.Chart = c;

            if (this.alarm_detected == 1)
            {
                this.AlarmDetected.Color = System.Drawing.Color.Red;
            }
            else
            {
                this.AlarmDetected.Color = System.Drawing.Color.Green;
            }
        }

        private void START_Click(object sender, EventArgs e)
        {
            if (Serial_port_open == true)
            {
                STOP.Visible = true;
                START.Visible = false;
                Start_acquisition = true;
                Threshold_Limit.Enabled = false;
                Set_nSample.Enabled = false;
                Gain_selector.Enabled = false;
                Soglia_intervento.Enabled = false;
                SAVE_PARAM.Visible = true;
                SAVE_PARAM.Enabled = false;


                pkt = new packet();
                packet_length = Marshal.SizeOf(pkt);
                buffer = new byte[packet_length];
                pkt.cmd = Command.CMD_READ;
                pkt.sub_cmd = Sub_Command.READ_SENSOR;
                pkt.STX = Command.STX_CONST;
                pkt.ETX = Command.ETX_CONST;
                serialPort1.Write(getBytes(pkt), 0, packet_length);
            }
            else
            {
                //
            }
        }

        private void STOP_Click(object sender, EventArgs e)
        {
            STOP.Visible = false;
            START.Visible = true;
            Start_acquisition = false;
            Threshold_Limit.Enabled = true;
            Set_nSample.Enabled = true;
            Gain_selector.Enabled = true;
            SAVE_PARAM.Enabled = true;
            Soglia_intervento.Enabled = true;
            gainScrollBar.Enabled = true;
        }

        private void Set_nSample_Click(object sender, EventArgs e)
        {

            nSampleValue = Convert.ToUInt16(this.nSample_Value.Text);

            if (Serial_port_open == true)
            {
                if (Start_acquisition == false)
                {
                    pkt = new packet();
                    packet_length = Marshal.SizeOf(pkt);
                    buffer = new byte[packet_length];
                    pkt.cmd = Command.CMD_WRITE;
                    pkt.STX = Command.STX_CONST;
                    pkt.ETX = Command.ETX_CONST;
                    pkt.sub_cmd = Sub_Command.SET_NCAMP;


                    pkt.payload = new byte[PL_LENGTH];
                    pkt.payload[0] = (byte)nSampleValue;
                    pkt.payload[1] = (byte)(nSampleValue>>8);

                    serialPort1.Write(getBytes(pkt), 0, packet_length);
                }
            }
        }

        private void Threshold_Limit_Scroll(object sender, ScrollEventArgs e)
        {
            UInt16 Threshold_limit = (UInt16)(Threshold_Limit.Maximum - Threshold_Limit.Value);
            threshold_value.Text = Threshold_limit.ToString();
            if ((serialPort1.IsOpen == true)&&
                (Start_acquisition == false)&&
                (threshold_cmd_sem == true))
            {
                    pkt = new packet();
                    packet_length = Marshal.SizeOf(pkt);
                    buffer = new byte[packet_length];
                    pkt.cmd = Command.CMD_WRITE;
                    pkt.STX = Command.STX_CONST;
                    pkt.ETX = Command.ETX_CONST;
                    pkt.sub_cmd = Sub_Command.SET_THSD;


                    pkt.payload = new byte[PL_LENGTH];
                    pkt.payload[0] = (byte)Threshold_limit;
                    pkt.payload[1] = (byte)(Threshold_limit>>8);

                    serialPort1.Write(getBytes(pkt), 0, packet_length);
            }
            else
            {
                Threshold_Limit.Value = Old_threshold;
            }
        }

        private void comboBox1_SelectedIndexChanged(object sender, EventArgs e)
        {
            int Gain_index = Gain_selector.SelectedIndex + 1;
            byte Gain_value = Convert.ToByte(Gain_index);

            if ((serialPort1.IsOpen == true) && (Start_acquisition == false))
            {
                pkt = new packet();
                packet_length = Marshal.SizeOf(pkt);
                buffer = new byte[packet_length];
                pkt.cmd = Command.CMD_WRITE;
                pkt.STX = Command.STX_CONST;
                pkt.ETX = Command.ETX_CONST;
                pkt.sub_cmd = Sub_Command.SET_GAIN_1;


                pkt.payload = new byte[PL_LENGTH];
                pkt.payload[0] = Gain_value;

                serialPort1.Write(getBytes(pkt), 0, packet_length);
            }
            else
            {
            }
        }

        private void SAVE_PARAM_Click(object sender, EventArgs e)
        {

            if (Serial_port_open == true)
            {
                if (Start_acquisition == false)
                {
                    pkt = new packet();
                    packet_length = Marshal.SizeOf(pkt);
                    buffer = new byte[packet_length];
                    pkt.cmd = Command.CMD_WRITE;
                    pkt.STX = Command.STX_CONST;
                    pkt.ETX = Command.ETX_CONST;
                    pkt.sub_cmd = Sub_Command.SAVE_PAR;


                    pkt.payload = new byte[PL_LENGTH];
                    pkt.payload[0] = (byte)0x00;
                    pkt.payload[1] = (byte)0x00;

                    serialPort1.Write(getBytes(pkt), 0, packet_length);
                }
            }

        }

        private void Soglia_intervento_Click(object sender, EventArgs e)
        {
            soglia_intervento = Convert.ToUInt16(this.textBox1.Text);

            if (Serial_port_open == true)
            {
                if (Start_acquisition == false)
                {
                    pkt = new packet();
                    packet_length = Marshal.SizeOf(pkt);
                    buffer = new byte[packet_length];
                    pkt.cmd = Command.CMD_WRITE;
                    pkt.STX = Command.STX_CONST;
                    pkt.ETX = Command.ETX_CONST;
                    pkt.sub_cmd = Sub_Command.SET_SOGLIA;


                    pkt.payload = new byte[PL_LENGTH];
                    pkt.payload[0] = (byte)soglia_intervento;
                    pkt.payload[1] = (byte)(soglia_intervento >> 8);

                    serialPort1.Write(getBytes(pkt), 0, packet_length);
                }
            }
        }

        private void vScrollBar1_Scroll(object sender, ScrollEventArgs e)
        {

            UInt16 gain_limit = (UInt16)(gainScrollBar.Maximum - gainScrollBar.Value);
            if ((serialPort1.IsOpen == true) &&
                (Start_acquisition == false) &&
                (threshold_cmd_sem == true))
            {
                pkt = new packet();
                packet_length = Marshal.SizeOf(pkt);
                buffer = new byte[packet_length];
                pkt.cmd = Command.CMD_WRITE;
                pkt.STX = Command.STX_CONST;
                pkt.ETX = Command.ETX_CONST;
                pkt.sub_cmd = Sub_Command.SET_GAIN_2;


                pkt.payload = new byte[PL_LENGTH];
                pkt.payload[0] = (byte)gain_limit;
                pkt.payload[1] = (byte)0x00;

                serialPort1.Write(getBytes(pkt), 0, packet_length);
            }
        }

  
    }
}
