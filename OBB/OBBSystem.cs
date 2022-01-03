using System;
using Rhino;
using Rhino.Commands;
using Rhino.Geometry;
using Rhino.Collections;
using System.Collections.Generic;

namespace OBB
{
    public class OBBSystem
    {
        public int I;
        public System.Collections.Generic.List<GeometryBase> objs;

        // multi-object planarity/coplanarity check function
        public Plane CheckObjCoPlanarity(List<GeometryBase> objs)
        {
            List<Point3d> ptlist = new List<Point3d>();
            Plane myplane = new Plane();
            bool myresult = new bool();
            RhinoDoc rd = RhinoDoc.ActiveDoc;
            double tol = rd.ModelAbsoluteTolerance;
            foreach (var obj in objs)
            {
                if (obj is Rhino.Geometry.Curve)
                {
                    Curve objcurve = (Curve)obj;
                    myresult = objcurve.TryGetPlane(out myplane, tol);

                    if (myresult == false)
                    {
                        myplane = default(Plane);
                        return myplane;
                    }

                    NurbsCurve mynurbscurve = objcurve.ToNurbsCurve();

                    if (mynurbscurve == default(NurbsCurve))
                    {
                        myplane = default(Plane);
                        return myplane;
                    }

                    for (int i = 0; i < mynurbscurve.Points.Count; i = i + 1)
                    {

                        ptlist.Add(mynurbscurve.Points[i].Location);

                    }

                }

                else if (obj is Rhino.Geometry.Point)
                {
                    Point objpoint = (Point)obj;
                    ptlist.Add(objpoint.Location);

                }

                else if (obj is Rhino.Geometry.PointCloud)
                {
                    PointCloud objpointcloud = (PointCloud)obj;
                    Point3d[] ptarray = objpointcloud.GetPoints();
                    foreach (Point3d i in ptarray)
                    {
                        ptlist.Add(i);
                    }

                }

                else if (obj is Rhino.Geometry.Brep)
                {
                    Brep objbrep = (Brep)obj;
                    foreach (BrepFace face in objbrep.Faces)
                    {
                        myresult = face.TryGetPlane(out myplane, tol);

                        if (myresult == false)
                        {
                            myplane = default(Plane);
                            return myplane;
                        }
                        NurbsSurface mysurfaceface = face.ToNurbsSurface();

                        if (mysurfaceface == default(NurbsSurface))
                        {
                            myplane = default(Plane);
                            return myplane;
                        }

                        foreach (ControlPoint cp in mysurfaceface.Points)
                        {
                            ptlist.Add(cp.Location);
                        }

                    }
                }

                else if (obj is Rhino.Geometry.Surface)
                {
                    Surface objsurface = (Surface)obj;
                    myresult = objsurface.TryGetPlane(out myplane, tol);
                    if (myresult == false)
                    {
                        myplane = default(Plane);
                        return myplane;
                    }

                    NurbsSurface mysurfaceface = objsurface.ToNurbsSurface();

                    if (mysurfaceface == default(NurbsSurface))
                    {
                        myplane = default(Plane);
                        return myplane;
                    }


                    foreach (ControlPoint cp in mysurfaceface.Points)
                    {
                        ptlist.Add(cp.Location);
                    }



                }

                else if (obj is Rhino.Geometry.Extrusion)
                {
                    Extrusion objextrusion = (Extrusion)obj;
                    myresult = objextrusion.TryGetPlane(out myplane, tol);
                    if (myresult == false)
                    {
                        myplane = default(Plane);
                        return myplane;
                    }

                    NurbsSurface mysurfaceface = objextrusion.ToNurbsSurface();
                    if (mysurfaceface == default(NurbsSurface))
                    {
                        myplane = default(Plane);
                        return myplane;
                    }
                    foreach (ControlPoint cp in mysurfaceface.Points)
                    {
                        ptlist.Add(cp.Location);
                    }

                }

                else if (obj is Rhino.Geometry.Mesh)
                {
                    Mesh objmesh = (Mesh)obj;
                    foreach (Point3d vert in objmesh.Vertices)
                    {
                        ptlist.Add(vert);

                    }

                }

                else
                {
                    return myplane;
                }

            }
            Plane.FitPlaneToPoints(ptlist, out myplane);
            return myplane;
        }

        // gets a plane-aligned bounding box
        public Box BoundingBoxPlane(List<GeometryBase> objs, Plane plane, bool accurate)
        {
            accurate = true;
            Plane wxy_plane = Rhino.Geometry.Plane.WorldXY;
            Transform xform = Rhino.Geometry.Transform.ChangeBasis(wxy_plane, plane);
            BoundingBox bbox = Rhino.Geometry.BoundingBox.Empty;

            foreach (GeometryBase obj in objs)
            {
                BoundingBox objectbbox = Objectbbox(obj, xform, accurate);
                bbox = Rhino.Geometry.BoundingBox.Union(bbox, objectbbox);
            }

            if (bbox.IsValid != true)         //not sure its necessary
            {
                Box emptybox = Rhino.Geometry.Box.Empty;
                return emptybox;
            }

            Transform plane_to_world = Rhino.Geometry.Transform.ChangeBasis(plane, wxy_plane);


            Box box = new Rhino.Geometry.Box(bbox);
            box.Transform(plane_to_world);
            return box;
        }


        // gets a plane-aligned bounding box - inside method
        public BoundingBox Objectbbox(GeometryBase geom, Transform xform, bool accurate)
        {
            if (geom is Rhino.Geometry.Point)
            {
                Point geompt = (Point)geom;
                Point3d geompt3d = geompt.Location;
                if (xform != null)
                {
                    geompt3d = xform * geompt3d;

                    BoundingBox geompt3dbbox = new Rhino.Geometry.BoundingBox(geompt3d, geompt3d);
                    return geompt3dbbox;
                }
            }

            if (xform != null)
            {
                return geom.GetBoundingBox(xform);
            }
            return geom.GetBoundingBox(accurate);
        }

        // used in initial 3D bb calculation
        public List<Plane> RotateCopyPlanes(double tot_ang, int count, List<Plane> init_planes, Vector3d dir_vec)
        {
            double inc = tot_ang / (count - 1);
            Point3d origin = new Rhino.Geometry.Point3d(0, 0, 0);
            List<Plane> planes = new List<Plane>();
            for (int i = 0; i < count; i++)
            {
                foreach (Plane init_plane in init_planes)
                {
                    Plane new_plane = new Rhino.Geometry.Plane(init_plane);
                    new_plane.Rotate(inc * i, dir_vec, origin);
                    planes.Add(new_plane);
                }
            }
            return planes;
        }

        // used in initial 3D bb calculation

        public List<Plane> GenerateOctantPlane(int count)
        {
            double tot_ang = Math.PI * 0.5;
            // generates an array of count^3 planes in 3 axes covering xyz positive octant
            Plane yz_plane = Rhino.Geometry.Plane.WorldYZ;
            List<Plane> yz_planelist = new List<Plane>();
            yz_planelist.Add(yz_plane);
            Vector3d Xdir_vec = new Rhino.Geometry.Vector3d(1, 0, 0); //X axis
            List<Plane> x_planes = RotateCopyPlanes(tot_ang, count, yz_planelist, Xdir_vec);


            Vector3d Ydir_vec = new Rhino.Geometry.Vector3d(0, -1, 0); //-Y axis
            List<Plane> xy_planes = RotateCopyPlanes(tot_ang, count, x_planes, Ydir_vec);

            Vector3d Zdir_vec = new Rhino.Geometry.Vector3d(0, 0, 1); //Z axis
            List<Plane> xyz_planes = RotateCopyPlanes(tot_ang, count, xy_planes, Zdir_vec);

            return xyz_planes;

        }

        //used in 3D refinement calculation

        public List<Plane> RotatedPlaneArray(Plane plane, double tot_ang, int divs, Vector3d axis)
        {
            List<Plane> out_plane = new List<Plane>();
            plane.Rotate(-tot_ang * 0.5, axis);
            out_plane.Add(new Rhino.Geometry.Plane(plane));
            double inc = tot_ang / (divs - 1);
            for (int i = 0; i < divs - 1; i++)
            {
                plane.Rotate(inc, axis);
                out_plane.Add(new Rhino.Geometry.Plane(plane));
            }
            return out_plane;
        }


        public List<Plane> RotatePlaneArray3D(Plane view_plane, double tot_ang, int divs)
        {
            //generate a 3D array of refinement planes (works with narrow angles)
            List<Plane> out_planes = new List<Plane>();
            //use RotatedPlaneArray to generate 'horizontal' left-right array (yaw)
            List<Plane> yaw_planes = RotatedPlaneArray(view_plane, tot_ang, divs, view_plane.ZAxis);
            foreach (Plane y_plane in yaw_planes)
            {
                //use RotatedPlaneArray to generate up-down 'tilt' array(roll)
                List<Plane> roll_planes = RotatedPlaneArray(y_plane, tot_ang, divs, y_plane.YAxis);

                foreach (Plane r_plane in roll_planes)
                {
                    //use RotatedPlaneArray to generate up-down 'tilt' array(pitch)
                    List<Plane> pitch_planes = RotatedPlaneArray(r_plane, tot_ang, divs, r_plane.XAxis);

                    foreach (Plane p_plane in pitch_planes)
                    {
                        out_planes.Add(p_plane);
                    }

                }
            }
            return out_planes;
        }


        //this is the main 3D bb calculation search function
        public void MinBBPlane(List<GeometryBase> objs, ref Plane best_plane, List<Plane> planes, ref Box curr_box, ref double curr_vol)
        {



            foreach (Plane plane in planes)
            {
                Box bb = BoundingBoxPlane(objs, plane,true);

                if (bb.Volume < curr_vol)
                {
                    curr_vol = bb.Volume;
                    best_plane = plane;
                    curr_box = bb;
                }

            }

        }

        //3D (non-planar) bounding box routine
        public void Min3DBoundingBox(List<GeometryBase> objs, Plane init_plane, int count, bool rel_stop, bool im_rep, out Box curr_bb, out double curr_vol, out int passes, out List<string> insidemsg)
        {
            // for non-planar or non-coplanar object(s)
            // get initial fast bb in init plane (World XY), plus volume to compare
            //this is the main 2D bb calculation search function

            curr_bb = BoundingBoxPlane(objs, init_plane,true);

            curr_vol = curr_bb.Volume;

            double tot_ang = Math.PI * 0.5;  //90 degrees for intial octant
            double factor = 0.1; //angle reduction factor for each successive refinement pass
            int max_passes = 20; //safety factor
            int prec = Rhino.RhinoDoc.ActiveDoc.DistanceDisplayPrecision;
            string us = Rhino.RhinoDoc.ActiveDoc.GetUnitSystemName(true, false, false, false);
            insidemsg = new List<string>();
            //run intitial bb calculation
            List<Plane> xyz_planes = GenerateOctantPlane(count);
            Plane best_plane = new Plane();
            MinBBPlane(objs, ref best_plane, xyz_planes, ref curr_bb, ref curr_vol);
            //report the results of intial rough calculation
            if (im_rep == true)
            {
                double roundcurr_vol = System.Math.Round(curr_vol, prec);
                insidemsg.Add("Initial pass 0, volume: " + roundcurr_vol.ToString() + " cu " + us.ToString());
            }
            //refine with smaller angles arround best fit plane, loop until...
            int i = 0;
            for (i = 0; i < max_passes; i++)
            {
                double prev_vol = curr_vol;

                //reduce angle by factor, use refinement planes to generate array

                tot_ang *= factor;
                List<Plane> ref_planes = RotatePlaneArray3D(best_plane, tot_ang, count);
                MinBBPlane(objs, ref best_plane, ref_planes, ref curr_bb, ref curr_vol);

                double vol_diff = prev_vol - curr_vol;//vol. diff. over last pass, should be positive or 0
                if (im_rep == true)
                {
                    insidemsg.Add("Volume difference from last pass:  " + vol_diff.ToString());
                }
                // check if difference is less than minimum "significant"
                // rel_stop==True: relative stop value <.01% difference from previous

                if (rel_stop == true)
                {
                    if (vol_diff < 0.0001 * prev_vol)
                    {
                        break;
                    }
                }
                else
                {
                    RhinoDoc rd = RhinoDoc.ActiveDoc;
                    if (vol_diff < rd.ModelAbsoluteTolerance)
                    {
                        break;
                    }
                }
                Rhino.RhinoApp.Wait();

                if (im_rep == true)
                {
                    int k = i + 1;
                    double roundcurr_vol = System.Math.Round(curr_vol, prec);
                    insidemsg.Add("Refine pass " + k.ToString() + "  volume: " + roundcurr_vol.ToString() + " cu " + us.ToString());
                }

                //get out of loop if escape is pressed

                //if (Console.ReadKey().Key != ConsoleKey.Escape)
                //{
                //Console.WriteLine("Refinement aborted after {0} passes", i + 1);
                //break;
                //}
            }
            passes = i + 1;


        }

        public void PlanarMinBB(List<GeometryBase> objs, Plane plane, double tot_ang, int divs, out Plane curr_plane, out double curr_area)
        {
            double inc = tot_ang / divs;
            //rotate plane half total angle minus direction
            plane.Rotate(-tot_ang * 0.5, plane.ZAxis, plane.Origin);
            Box bb = BoundingBoxPlane(objs, plane, true);
            curr_plane = new Plane(plane);
            curr_area = BoxArea(bb);
            //loop through angle increments
            for (int i = 0; i < divs; i++)
            {
                plane.Rotate(inc, plane.ZAxis, plane.Origin);
                bb = BoundingBoxPlane(objs, plane, true);
                double new_area = BoxArea(bb);
                if (new_area < curr_area)
                {
                    curr_area = new_area;
                    curr_plane = plane;
                }
            }


        }

        //2D planar bounding rectangle routine
        public void MinBoundingRectanglePlane(List<GeometryBase> objs, Plane curr_plane, bool im_rep, out Box f_bb, out Double curr_area, out int i, out List<string> insidemsg)
        {
            //pass True argument above if you want to print intermediate results
            //initialize
            double factor = 0.01;  //0.01 (1/100 of model abs tolerance)
            double angle = Math.PI * 0.5; //start angle 90 degrees
            int divs = 90;  //initial division 1 degree intervals
            RhinoDoc rd = RhinoDoc.ActiveDoc;
            double tol = rd.ModelAbsoluteTolerance;
            insidemsg = new List<string>();
            string err_msg = "Unable to calculate bouding box area";


            f_bb = new Box();
            i = 0;





            ///get initial rough bounding box

            Box init_bb = BoundingBoxPlane(objs, curr_plane, false);
            curr_area = BoxArea(init_bb);
            if (im_rep == true)
            {
                insidemsg.Add("Initial area:" + curr_area.ToString());
            }

            //main calculation loop
            int safe = 10;

            for (i = 0; i < safe; i++)
            {

                PlanarMinBB(objs, curr_plane, angle, divs, out curr_plane, out curr_area);
                double new_area = curr_area;
                //abort if area is 0 or extremly small
                if (new_area < tol * 0.1)
                {
                    insidemsg.Add(err_msg);
                    return;
                }
                //break out of loop if new area is the smae as prev. area within limit
                if (Math.Abs(curr_area - new_area) < factor * tol)
                {

                    break;
                }
                //otherwise,decrease increments and loop
                curr_area = new_area;
                angle *= (1 / divs);

                if (im_rep == true)
                {
                    int k = i + 1;
                    insidemsg.Add("Refine stage" + k.ToString() + "Area:" + curr_area.ToString());
                    Rhino.RhinoApp.Wait();//wait for command line to print...

                }
                if (i == 10)
                {
                    insidemsg.Add("Max loop limit reached");//debug
                }
            }

            f_bb = BoundingBoxPlane(objs, curr_plane, true);


        }

        //used for planar bounding rectangle calculation
        public double BoxArea(Box box)
        {
            return (box.X[1] - box.X[0]) * (box.Y[1] - box.Y[0]);
        }

        //Main 
        public Box CombinedMinBBMulti(List<GeometryBase> objs, int fine_sample, bool im_rep, out List<string> msg, out Box bb)
        {
            msg = new List<string>();
            int prec = Rhino.RhinoDoc.ActiveDoc.DistanceDisplayPrecision;
            string us = Rhino.RhinoDoc.ActiveDoc.GetUnitSystemName(true, false, false, false);

            List<GeometryBase> inputs = objs;
            int count = fine_sample;
            int passes = 0;
            bb = Box.Empty;
            List<string> insidemsg = new List<string>();


            msg.Add("checking object planarity/coplanarity...\r\n");

            DateTime starttime = System.DateTime.Now;


            Plane plane = CheckObjCoPlanarity(objs);

            if (plane != default(Plane))
            {
                if (objs.Count == 1)
                {
                    msg.Add("Selected object is planar -\r\n");
                }
                else
                {
                    msg.Add("All selected objects are coplanar - \r\n");
                    msg.Add("launching 2D planar bounding rectangle calculation.\r\n");
                }


                //launch planar bounding box routine

                Box f_bb = Box.Empty;
                Point3d[] f_bbpt = Box.Empty.GetCorners();
                double curr_area = 0.0;
                int i = 0;


                MinBoundingRectanglePlane(objs, plane, im_rep, out f_bb, out curr_area, out i, out insidemsg);

                foreach (string x in insidemsg)
                {
                    msg.Add(x);
                }

                passes = i;


                bb = f_bb;

                double fa = Math.Round(curr_area, prec);


                msg.Add(passes.ToString() + " refinement stages\r\n");
                msg.Add("Minimum bounding box area = " + fa.ToString() + " sq " + us.ToString() + "\r\n");

                DateTime endtime = System.DateTime.Now;
                double costtime = (endtime - starttime).TotalMilliseconds;

                msg.Add("Elapsed time: " + costtime.ToString() + " Milliseconds");


            }

            else
            {
                // standard sample count=10 --> 1000 boxes per pass
                // fine sample count=18 --> 5832 boxes per pass     //dont use here




                if (objs.Count == 1)
                {
                    msg.Add("Selected object is not planar -\r\n");
                }

                else
                {
                    msg.Add("Selected object are not planar -\r\n");
                }

                msg.Add("launching 3D bounding box calculation.\r\n");
                double curr_vol = 0.0;
                Plane wxy_plane = Rhino.Geometry.Plane.WorldXY;
                Box curr_bb = Box.Empty;

                Min3DBoundingBox(objs, wxy_plane, count, false, im_rep, out curr_bb, out curr_vol, out passes, out insidemsg);
                foreach (string x in insidemsg)
                {
                    msg.Add(x);
                }

                bb = curr_bb;
                double fv = Math.Round(curr_vol, prec);
                msg.Add("Final volume after " + passes.ToString() + " passes is " + fv.ToString() + " sq3 " + us.ToString());

                DateTime endtime = System.DateTime.Now;
                double costtime = (endtime - starttime).TotalMilliseconds;

                msg.Add("Elapsed time: " + costtime.ToString() + " Milliseconds");


            }
            return bb;

        }
    }
}