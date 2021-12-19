using Grasshopper;
using Grasshopper.Kernel;
using System;
using System.Drawing;

namespace OBB
{
    public class OBBInfo : GH_AssemblyInfo
    {
        public override string Name => "OBB";

        //Return a 24x24 pixel bitmap to represent this GHA library.
        public override Bitmap Icon
        {
            get
            {
                return Properties.Resources.OBB_icon;
            }
        }

        //Return a short string describing the purpose of this GHA library.
        public override string Description => "";

        public override Guid Id => new Guid("01314A71-E57E-4DF3-8789-69211027086B");

        //Return a string identifying you or your company.
        public override string AuthorName => "";

        //Return a string representing your preferred contact details.
        public override string AuthorContact => "";
    }
}