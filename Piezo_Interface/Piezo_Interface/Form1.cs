using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System;
using System.IO.Ports;
using GraphLib;

namespace Piezo_Interface
{
    public partial class Form1 : Form
    {
        PrecisionTimer.Timer myTimer = null;
        int point = 0;
        public Form1()
        {
            //myTimer = new PrecisionTimer.Timer();
            //myTimer.period
            InitializeComponent();

            display.Smoothing = System.Drawing.Drawing2D.SmoothingMode.None;

            this.Load += Form1_Load;
            CalcDataGraphs("0");
            display.Refresh();

        }
        private void button1_Click(object sender, EventArgs e)
        {
            string portName = SerialPort_List.SelectedItem.ToString();
            
            var serialPort1 = new SerialPort(portName);
           // var serialPort1 = new SerialPort("USBSER000");

            serialPort1.Parity = Parity.None;
            serialPort1.StopBits = StopBits.One;
            serialPort1.DataBits = 8;
            serialPort1.Handshake = Handshake.None;
            serialPort1.DataReceived += new System.IO.Ports.SerialDataReceivedEventHandler(DataReceivedHandler);
            
            serialPort1.BaudRate = 38400;

            try
            {
                serialPort1.Open();
                serialPort1.Write("\n\r");
            }
            catch
            {
                Console.WriteLine("serialPort not opened");
            }
        }

        private void DataReceivedHandler(object sender, SerialDataReceivedEventArgs e)
        {
            SerialPort sp = (SerialPort)sender;
            string indata = sp.ReadLine();
            //int indata = sp.ReadLine();

            CalcDataGraphs(indata);
            //point++;

            this.Invoke(new MethodInvoker(RefreshGraph));
            Console.WriteLine("Data Received:");
            Console.Write(indata);
        }

        void Form1_Load(object sender, EventArgs e)
        {
            var ports = SerialPort.GetPortNames();
            SerialPort_List.DataSource = ports;
            SerialPort_List.Items.AddRange(ports);
        }

        void fill_data(DataSource ds, int idx, string str)
        {
        cPoint[] src = ds.Samples;
            for(int i=0;i<src.Length;i++)
            {
            src[i].x = i;
            src[i].y = float.Parse(str); 
            }
        }
        
        
        protected void CalcDataGraphs(string str)
        {

            this.SuspendLayout();

            display.DataSources.Clear();
            display.SetDisplayRangeX(0, 400);

                display.DataSources.Add(new DataSource());
                display.DataSources[0].Name = "Graph " + (1);
                display.DataSources[0].OnRenderXAxisLabel += RenderXLabel;




                switch ("NORMAL")
                {
                    case "NORMAL":
                        this.Text = "Normal Graph";
                        display.DataSources[0].Length = 5800;
                        display.PanelLayout = PlotterGraphPaneEx.LayoutMode.NORMAL;
                        display.DataSources[0].AutoScaleY = false;
                        display.DataSources[0].SetDisplayRangeY(-300, 300);
                        display.DataSources[0].SetGridDistanceY(100);
                        display.DataSources[0].OnRenderYAxisLabel = RenderYLabel;
                        //CalcSinusFunction_0(display.DataSources[j], j);
                        break;

                    case "NORMAL_AUTO":
                        this.Text = "Normal Graph Autoscaled";
                        display.DataSources[0].Length = 5800;
                        display.PanelLayout = PlotterGraphPaneEx.LayoutMode.NORMAL;
                        display.DataSources[0].AutoScaleY = true;
                        display.DataSources[0].SetDisplayRangeY(-300, 300);
                        display.DataSources[0].SetGridDistanceY(100);
                        display.DataSources[0].OnRenderYAxisLabel = RenderYLabel;
                        //CalcSinusFunction_0(display.DataSources[j], j);
                        break;

                    case "STACKED":
                        this.Text = "Stacked Graph";
                        display.PanelLayout = PlotterGraphPaneEx.LayoutMode.STACKED;
                        display.DataSources[0].Length = 5800;
                        display.DataSources[0].AutoScaleY = false;
                        display.DataSources[0].SetDisplayRangeY(-250, 250);
                        display.DataSources[0].SetGridDistanceY(100);
                        //CalcSinusFunction_1(display.DataSources[j], j);
                        break;

                    case "VERTICAL_ALIGNED":
                        this.Text = "Vertical aligned Graph";
                        display.PanelLayout =
                            PlotterGraphPaneEx.LayoutMode.VERTICAL_ARRANGED;
                        display.DataSources[0].Length = 5800;
                        display.DataSources[0].AutoScaleY = false;
                        display.DataSources[0].SetDisplayRangeY(-300, 300);
                        display.DataSources[0].SetGridDistanceY(100);
                        //CalcSinusFunction_2(display.DataSources[j], j);
                        break;

                    case "VERTICAL_ALIGNED_AUTO":
                        this.Text = "Vertical aligned Graph autoscaled";
                        display.PanelLayout =
                            PlotterGraphPaneEx.LayoutMode.VERTICAL_ARRANGED;
                        display.DataSources[0].Length = 5800;
                        display.DataSources[0].AutoScaleY = true;
                        display.DataSources[0].SetDisplayRangeY(-300, 300);
                        display.DataSources[0].SetGridDistanceY(100);
                        //CalcSinusFunction_2(display.DataSources[j], j);
                        break;

                    case "TILED_VERTICAL":
                        this.Text = "Tiled Graphs (vertical prefered)";
                        display.PanelLayout = PlotterGraphPaneEx.LayoutMode.TILES_VER;
                        display.DataSources[0].Length = 5800;
                        display.DataSources[0].AutoScaleY = false;
                        display.DataSources[0].SetDisplayRangeY(-300, 600);
                        display.DataSources[0].SetGridDistanceY(100);
                        //CalcSinusFunction_2(display.DataSources[j], j);
                        break;

                    case "TILED_VERTICAL_AUTO":
                        this.Text = "Tiled Graphs (vertical prefered) autoscaled";
                        display.PanelLayout = PlotterGraphPaneEx.LayoutMode.TILES_VER;
                        display.DataSources[0].Length = 5800;
                        display.DataSources[0].AutoScaleY = true;
                        display.DataSources[0].SetDisplayRangeY(-300, 600);
                        display.DataSources[0].SetGridDistanceY(100);
                        //CalcSinusFunction_2(display.DataSources[j], j);
                        break;

                    case "TILED_HORIZONTAL":
                        this.Text = "Tiled Graphs (horizontal prefered)";
                        display.PanelLayout = PlotterGraphPaneEx.LayoutMode.TILES_HOR;
                        display.DataSources[0].Length = 5800;
                        display.DataSources[0].AutoScaleY = false;
                        display.DataSources[0].SetDisplayRangeY(-300, 600);
                        display.DataSources[0].SetGridDistanceY(100);
                        //CalcSinusFunction_2(display.DataSources[j], j);
                        break;

                    case "TILED_HORIZONTAL_AUTO":
                        this.Text = "Tiled Graphs (horizontal prefered) autoscaled";
                        display.PanelLayout = PlotterGraphPaneEx.LayoutMode.TILES_HOR;
                        display.DataSources[0].Length = 5800;
                        display.DataSources[0].AutoScaleY = true;
                        display.DataSources[0].SetDisplayRangeY(-300, 600);
                        display.DataSources[0].SetGridDistanceY(100);
                        //CalcSinusFunction_2(display.DataSources[j], j);
                        break;

                    case "ANIMATED_AUTO":

                        this.Text = "Animated graphs fixed x range";
                        display.PanelLayout = PlotterGraphPaneEx.LayoutMode.TILES_HOR;
                        display.DataSources[0].Length = 402;
                        display.DataSources[0].AutoScaleY = false;
                        display.DataSources[0].AutoScaleX = true;
                        display.DataSources[0].SetDisplayRangeY(-300, 500);
                        display.DataSources[0].SetGridDistanceY(100);
                        display.DataSources[0].XAutoScaleOffset = 50;
                        //CalcSinusFunction_3(display.DataSources[j], j, 0);
                        display.DataSources[0].OnRenderYAxisLabel = RenderYLabel;
                        break;
                }

                fill_data(display.DataSources[0], point, str);

            //ApplyColorSchema();

            this.ResumeLayout();
            

        }

        private void RefreshGraph()
        {

            display.Refresh();
        
        }

        private String RenderXLabel(DataSource s, int idx)
        {
            if (s.AutoScaleX)
            {
                int Value = (int)(s.Samples[idx].x);
                return "" + Value;
            }
            else
            {
                int Value = (int)(s.Samples[idx].x / 200);
                String Label = "" + Value + "\"";
                return Label;
            }
        }

        private String RenderYLabel(DataSource s, float value)
        {
            return String.Format("{0:0.0}", value);
        }


    }
}
