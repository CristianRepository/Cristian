


namespace Piezo_Interface
{
    partial class Form1
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            this.SerialPort_List = new System.Windows.Forms.ComboBox();
            this.button1 = new System.Windows.Forms.Button();
            this.winChartViewer1 = new ChartDirector.WinChartViewer();
            this.dataRateTimer = new System.Windows.Forms.Timer(this.components);
            this.contextMenuStrip1 = new System.Windows.Forms.ContextMenuStrip(this.components);
            this.serialPort1 = new System.IO.Ports.SerialPort(this.components);
            this.Threshold_Limit = new System.Windows.Forms.VScrollBar();
            this.START = new System.Windows.Forms.Button();
            this.STOP = new System.Windows.Forms.Button();
            this.Set_nSample = new System.Windows.Forms.Button();
            this.nSample_Value = new System.Windows.Forms.TextBox();
            this.Gain_selector = new System.Windows.Forms.ComboBox();
            this.Electronic_Gain_label = new System.Windows.Forms.Label();
            this.SAVE_PARAM = new System.Windows.Forms.Button();
            this.label1 = new System.Windows.Forms.Label();
            this.textBox1 = new System.Windows.Forms.TextBox();
            this.Soglia_intervento = new System.Windows.Forms.Button();
            this.label2 = new System.Windows.Forms.Label();
            this.threshold_value = new System.Windows.Forms.TextBox();
            this.gainScrollBar = new System.Windows.Forms.VScrollBar();
            this.AlarmDetected = new Bulb.LedBulb();
            this.XAcceleration_TextBox = new System.Windows.Forms.TextBox();
            this.YAcceleration_TextBox = new System.Windows.Forms.TextBox();
            this.ZAcceleration_TextBox = new System.Windows.Forms.TextBox();
            this.label3 = new System.Windows.Forms.Label();
            this.label4 = new System.Windows.Forms.Label();
            this.label5 = new System.Windows.Forms.Label();
            ((System.ComponentModel.ISupportInitialize)(this.winChartViewer1)).BeginInit();
            this.SuspendLayout();
            // 
            // SerialPort_List
            // 
            this.SerialPort_List.FormattingEnabled = true;
            this.SerialPort_List.Location = new System.Drawing.Point(12, 12);
            this.SerialPort_List.Name = "SerialPort_List";
            this.SerialPort_List.Size = new System.Drawing.Size(121, 21);
            this.SerialPort_List.TabIndex = 2;
            // 
            // button1
            // 
            this.button1.Location = new System.Drawing.Point(140, 13);
            this.button1.Name = "button1";
            this.button1.Size = new System.Drawing.Size(75, 23);
            this.button1.TabIndex = 3;
            this.button1.Text = "OPEN";
            this.button1.UseVisualStyleBackColor = true;
            this.button1.Click += new System.EventHandler(this.button1_Click);
            // 
            // winChartViewer1
            // 
            this.winChartViewer1.Location = new System.Drawing.Point(30, 39);
            this.winChartViewer1.Name = "winChartViewer1";
            this.winChartViewer1.Size = new System.Drawing.Size(800, 600);
            this.winChartViewer1.TabIndex = 4;
            this.winChartViewer1.TabStop = false;
            this.winChartViewer1.ZoomInCursor = System.Windows.Forms.Cursors.Default;
            this.winChartViewer1.ZoomOutCursor = System.Windows.Forms.Cursors.Default;
            this.winChartViewer1.ViewPortChanged += new ChartDirector.WinViewPortEventHandler(this.winChartViewer1_ViewPortChanged);
            // 
            // dataRateTimer
            // 
            this.dataRateTimer.Tick += new System.EventHandler(this.dataRateTimer_Tick);
            // 
            // contextMenuStrip1
            // 
            this.contextMenuStrip1.Name = "contextMenuStrip1";
            this.contextMenuStrip1.Size = new System.Drawing.Size(61, 4);
            // 
            // Threshold_Limit
            // 
            this.Threshold_Limit.Enabled = false;
            this.Threshold_Limit.Location = new System.Drawing.Point(1025, 103);
            this.Threshold_Limit.Maximum = 4095;
            this.Threshold_Limit.Minimum = 1;
            this.Threshold_Limit.Name = "Threshold_Limit";
            this.Threshold_Limit.Size = new System.Drawing.Size(30, 200);
            this.Threshold_Limit.SmallChange = 10;
            this.Threshold_Limit.TabIndex = 5;
            this.Threshold_Limit.Value = 1;
            this.Threshold_Limit.Scroll += new System.Windows.Forms.ScrollEventHandler(this.Threshold_Limit_Scroll);
            // 
            // START
            // 
            this.START.Location = new System.Drawing.Point(287, 10);
            this.START.Name = "START";
            this.START.Size = new System.Drawing.Size(75, 23);
            this.START.TabIndex = 6;
            this.START.Text = "START";
            this.START.UseVisualStyleBackColor = true;
            this.START.Visible = false;
            this.START.Click += new System.EventHandler(this.START_Click);
            // 
            // STOP
            // 
            this.STOP.Location = new System.Drawing.Point(368, 10);
            this.STOP.Name = "STOP";
            this.STOP.Size = new System.Drawing.Size(75, 23);
            this.STOP.TabIndex = 7;
            this.STOP.Text = "STOP";
            this.STOP.UseVisualStyleBackColor = true;
            this.STOP.Visible = false;
            this.STOP.Click += new System.EventHandler(this.STOP_Click);
            // 
            // Set_nSample
            // 
            this.Set_nSample.Enabled = false;
            this.Set_nSample.Location = new System.Drawing.Point(850, 78);
            this.Set_nSample.Name = "Set_nSample";
            this.Set_nSample.Size = new System.Drawing.Size(100, 23);
            this.Set_nSample.TabIndex = 10;
            this.Set_nSample.Text = "Set_nSample";
            this.Set_nSample.UseVisualStyleBackColor = true;
            this.Set_nSample.Click += new System.EventHandler(this.Set_nSample_Click);
            // 
            // nSample_Value
            // 
            this.nSample_Value.Location = new System.Drawing.Point(850, 52);
            this.nSample_Value.MaxLength = 4;
            this.nSample_Value.Name = "nSample_Value";
            this.nSample_Value.Size = new System.Drawing.Size(100, 20);
            this.nSample_Value.TabIndex = 12;
            this.nSample_Value.Text = "256";
            this.nSample_Value.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // Gain_selector
            // 
            this.Gain_selector.Enabled = false;
            this.Gain_selector.FormattingEnabled = true;
            this.Gain_selector.Items.AddRange(new object[] {
            "Gain = 1/7 ",
            "Gain = 1/3 ",
            "Gain = 1",
            "Gain = 1 + 2/3",
            "Gain = 3",
            "Gain == 4 + 1/3",
            "Gain = 7",
            "Gain  = 15"});
            this.Gain_selector.Location = new System.Drawing.Point(850, 417);
            this.Gain_selector.Name = "Gain_selector";
            this.Gain_selector.Size = new System.Drawing.Size(121, 21);
            this.Gain_selector.TabIndex = 1;
            this.Gain_selector.SelectedIndexChanged += new System.EventHandler(this.comboBox1_SelectedIndexChanged);
            // 
            // Electronic_Gain_label
            // 
            this.Electronic_Gain_label.AutoSize = true;
            this.Electronic_Gain_label.Location = new System.Drawing.Point(869, 228);
            this.Electronic_Gain_label.Name = "Electronic_Gain_label";
            this.Electronic_Gain_label.Size = new System.Drawing.Size(79, 13);
            this.Electronic_Gain_label.TabIndex = 14;
            this.Electronic_Gain_label.Text = "Electronic Gain";
            this.Electronic_Gain_label.Visible = false;
            // 
            // SAVE_PARAM
            // 
            this.SAVE_PARAM.Location = new System.Drawing.Point(855, 576);
            this.SAVE_PARAM.Name = "SAVE_PARAM";
            this.SAVE_PARAM.Size = new System.Drawing.Size(315, 62);
            this.SAVE_PARAM.TabIndex = 15;
            this.SAVE_PARAM.Text = "SAVE_PARAM";
            this.SAVE_PARAM.UseVisualStyleBackColor = true;
            this.SAVE_PARAM.Visible = false;
            this.SAVE_PARAM.Click += new System.EventHandler(this.SAVE_PARAM_Click);
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(863, 36);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(69, 13);
            this.label1.TabIndex = 16;
            this.label1.Text = "ACQ FILTER";
            // 
            // textBox1
            // 
            this.textBox1.Location = new System.Drawing.Point(855, 135);
            this.textBox1.MaxLength = 4;
            this.textBox1.Name = "textBox1";
            this.textBox1.Size = new System.Drawing.Size(100, 20);
            this.textBox1.TabIndex = 17;
            this.textBox1.Text = "10";
            this.textBox1.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // Soglia_intervento
            // 
            this.Soglia_intervento.Enabled = false;
            this.Soglia_intervento.Location = new System.Drawing.Point(855, 161);
            this.Soglia_intervento.Name = "Soglia_intervento";
            this.Soglia_intervento.Size = new System.Drawing.Size(100, 23);
            this.Soglia_intervento.TabIndex = 18;
            this.Soglia_intervento.Text = "SET";
            this.Soglia_intervento.UseVisualStyleBackColor = true;
            this.Soglia_intervento.Click += new System.EventHandler(this.Soglia_intervento_Click);
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(847, 119);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(119, 13);
            this.label2.TabIndex = 19;
            this.label2.Text = "SOGLIA INTERVENTO";
            // 
            // threshold_value
            // 
            this.threshold_value.Location = new System.Drawing.Point(992, 322);
            this.threshold_value.MaxLength = 4;
            this.threshold_value.Name = "threshold_value";
            this.threshold_value.Size = new System.Drawing.Size(100, 20);
            this.threshold_value.TabIndex = 20;
            this.threshold_value.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // gainScrollBar
            // 
            this.gainScrollBar.Enabled = false;
            this.gainScrollBar.LargeChange = 1;
            this.gainScrollBar.Location = new System.Drawing.Point(866, 251);
            this.gainScrollBar.Maximum = 31;
            this.gainScrollBar.Minimum = 1;
            this.gainScrollBar.Name = "gainScrollBar";
            this.gainScrollBar.Size = new System.Drawing.Size(84, 154);
            this.gainScrollBar.TabIndex = 31;
            this.gainScrollBar.Value = 1;
            this.gainScrollBar.Scroll += new System.Windows.Forms.ScrollEventHandler(this.vScrollBar1_Scroll);
            // 
            // AlarmDetected
            // 
            this.AlarmDetected.Location = new System.Drawing.Point(1014, 39);
            this.AlarmDetected.Name = "AlarmDetected";
            this.AlarmDetected.On = false;
            this.AlarmDetected.Size = new System.Drawing.Size(50, 49);
            this.AlarmDetected.TabIndex = 9;
            this.AlarmDetected.Text = "AlarmDetected";
            // 
            // XAcceleration_TextBox
            // 
            this.XAcceleration_TextBox.Location = new System.Drawing.Point(855, 488);
            this.XAcceleration_TextBox.Name = "XAcceleration_TextBox";
            this.XAcceleration_TextBox.Size = new System.Drawing.Size(100, 20);
            this.XAcceleration_TextBox.TabIndex = 32;
            // 
            // YAcceleration_TextBox
            // 
            this.YAcceleration_TextBox.Location = new System.Drawing.Point(964, 488);
            this.YAcceleration_TextBox.Name = "YAcceleration_TextBox";
            this.YAcceleration_TextBox.Size = new System.Drawing.Size(100, 20);
            this.YAcceleration_TextBox.TabIndex = 33;
            // 
            // ZAcceleration_TextBox
            // 
            this.ZAcceleration_TextBox.Location = new System.Drawing.Point(1070, 488);
            this.ZAcceleration_TextBox.Name = "ZAcceleration_TextBox";
            this.ZAcceleration_TextBox.Size = new System.Drawing.Size(100, 20);
            this.ZAcceleration_TextBox.TabIndex = 34;
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(867, 472);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(82, 13);
            this.label3.TabIndex = 35;
            this.label3.Text = "X - Acceleration";
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(973, 472);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(82, 13);
            this.label4.TabIndex = 36;
            this.label4.Text = "Y - Acceleration";
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(1078, 472);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(82, 13);
            this.label5.TabIndex = 37;
            this.label5.Text = "Z - Acceleration";
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(1240, 650);
            this.Controls.Add(this.label5);
            this.Controls.Add(this.label4);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.ZAcceleration_TextBox);
            this.Controls.Add(this.YAcceleration_TextBox);
            this.Controls.Add(this.XAcceleration_TextBox);
            this.Controls.Add(this.gainScrollBar);
            this.Controls.Add(this.threshold_value);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.Soglia_intervento);
            this.Controls.Add(this.textBox1);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.SAVE_PARAM);
            this.Controls.Add(this.Electronic_Gain_label);
            this.Controls.Add(this.Gain_selector);
            this.Controls.Add(this.nSample_Value);
            this.Controls.Add(this.Set_nSample);
            this.Controls.Add(this.AlarmDetected);
            this.Controls.Add(this.STOP);
            this.Controls.Add(this.START);
            this.Controls.Add(this.Threshold_Limit);
            this.Controls.Add(this.winChartViewer1);
            this.Controls.Add(this.button1);
            this.Controls.Add(this.SerialPort_List);
            this.Name = "Form1";
            this.Text = "Form1";
            ((System.ComponentModel.ISupportInitialize)(this.winChartViewer1)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.ComboBox SerialPort_List;
        private System.Windows.Forms.Button button1;
        private ChartDirector.WinChartViewer winChartViewer1;
        private System.Windows.Forms.Timer dataRateTimer;
        private System.Windows.Forms.ContextMenuStrip contextMenuStrip1;
        private System.IO.Ports.SerialPort serialPort1;
        private System.Windows.Forms.VScrollBar Threshold_Limit;
        private System.Windows.Forms.Button START;
        private System.Windows.Forms.Button STOP;
        private Bulb.LedBulb AlarmDetected;
        private System.Windows.Forms.Button Set_nSample;
        private System.Windows.Forms.TextBox nSample_Value;
        private System.Windows.Forms.ComboBox Gain_selector;
        private System.Windows.Forms.Label Electronic_Gain_label;
        private System.Windows.Forms.Button SAVE_PARAM;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.TextBox textBox1;
        private System.Windows.Forms.Button Soglia_intervento;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.TextBox threshold_value;
        private System.Windows.Forms.VScrollBar gainScrollBar;
        private System.Windows.Forms.TextBox XAcceleration_TextBox;
        private System.Windows.Forms.TextBox YAcceleration_TextBox;
        private System.Windows.Forms.TextBox ZAcceleration_TextBox;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.Label label5;
    }
}

