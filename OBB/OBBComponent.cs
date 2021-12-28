using Grasshopper;
using Grasshopper.Kernel;
using Rhino.Geometry;
using System;
using System.Collections.Generic;

namespace OBB
{
    public class OBBComponent : GH_Component
    {
        private OBBSystem myOBBSystem;
        /// <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>
        public OBBComponent()
          : base("Oriented Bounding Box", "OBB",
            "To create a oriented bounding box",
            "Utility", "Analysis")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGeometryParameter("Geometry","Geo","Geometry input",GH_ParamAccess.list);
            pManager.AddIntegerParameter("Iteration","I","The number of iteration",GH_ParamAccess.item);
            
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGeometryParameter("Oriented Bounding Box", "OBB", "Outcome", GH_ParamAccess.list);

        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object can be used to retrieve data from input parameters and 
        /// to store data in output parameters.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            
            List<GeometryBase> OBJS = new List<GeometryBase> ();
            int I = 0;
            List<Brep> bb = new List<Brep> ();

            DA.GetDataList(0, OBJS);
            DA.GetData("Iteration", ref I);

            var myobbsystem = new OBBSystem();
            bb= myobbsystem.CombinedMinBBMulti(OBJS, I);
            DA.SetDataList(0,bb);
            
            //=============================================================










        }

        

        /// <summary>
        /// Provides an Icon for every component that will be visible in the User Interface.
        /// Icons need to be 24x24 pixels.
        /// You can add image files to your project resources and access them like this:
        /// return Resources.IconForThisComponent;
        /// </summary>
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                return Properties.Resources.OBB_icon;
            }
        }


        /// <summary>
        /// Each component must have a unique Guid to identify it. 
        /// It is vital this Guid doesn't change otherwise old ghx files 
        /// that use the old ID will partially fail during loading.
        /// </summary>
        public override Guid ComponentGuid => new Guid("793D4B0C-C6E7-4014-B865-69214D13737F");
    }
}