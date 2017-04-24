/**
 * Copyright 2017 the University of Central Florida Research Foundation, Inc.
 * All rights reserved.
 * 
 *     Eugene M. Taranta II <etaranta@gmail.com>
 *     Amirreza Samiei <samiei@knights.ucf.edu>
 *     Mehran Maghoumi <mehran@cs.ucf.edu>
 *     Pooya Khaloo <pooya@cs.ucf.edu>
 *     Corey R. Pittman <cpittman@knights.ucf.edu>
 *     Joseph J. LaViola Jr. <jjl@cs.ucf.edu>
 * 
 * Subject to the terms and conditions of the Florida Public Educational
 * Institution non-exclusive software license, this software is distributed 
 * under a non-exclusive, royalty-free, non-sublicensable, non-commercial, 
 * non-exclusive, academic research license, and is distributed without warranty
 * of any kind express or implied. 
 *
 * The Florida Public Educational Institution non-exclusive software license
 * is located at <https://github.com/ISUE/Jackknife/blob/master/LICENSE>.
 */

using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;

namespace Jackknife.Evaluate
{
    public class RecordingReader
    {
        BinaryReader _bstream;

        long _frame_cnt;

        public RecordingReader(string selectedFilePath)
        {
            _bstream = new BinaryReader(
                new FileStream(
                    selectedFilePath,
                    FileMode.Open));

            long length = _bstream.BaseStream.Length;
            _frame_cnt = length / ((5 + 63) * 4);
        }

        public float ReadFloat()
        {
            return _bstream.ReadSingle();
        }

        public int ReadInt()
        {
            return _bstream.ReadInt32();
        }

        public long FrameCount()
        {
            return _frame_cnt;
        }

        public void Close()
        {
            _bstream.Close();
        }
    }
}
