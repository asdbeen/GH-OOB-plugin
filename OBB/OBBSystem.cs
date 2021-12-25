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

            foreach (var obj in objs)
            {
                if (obj is Rhino.Geometry.Curve)
                {
                    Curve objcurve = (Curve)obj;
                    myresult = objcurve.TryGetPlane(out myplane);

                    //if (myresult == false)
                    //{
                    //Print("curve.TryGetPlane failed");
                    //return myplane;
                    //}
                    NurbsCurve mynurbscurve = objcurve.ToNurbsCurve();


                    for (int i = 0; i < mynurbscurve.Points.Count; i = i + 1)
                    {

                        ptlist.Add(mynurbscurve.Points[i].Location);

                    }
                    Print("Here add a Curve");
                }

                else if (obj is Rhino.Geometry.Point)
                {
                    Point objpoint = (Point)obj;
                    ptlist.Add(objpoint.Location);
                    Print("Here add a Point");
                }

                else if (obj is Rhino.Geometry.PointCloud)
                {
                    PointCloud objpointcloud = (PointCloud)obj;
                    Point3d[] ptarray = objpointcloud.GetPoints();
                    foreach (Point3d i in ptarray)
                    {
                        ptlist.Add(i);
                    }
                    Print("Here add a PointCloud");
                }

                else if (obj is Rhino.Geometry.Brep)
                {
                    Brep objbrep = (Brep)obj;
                    foreach (BrepFace face in objbrep.Faces)
                    {
                        myresult = face.TryGetPlane(out myplane);

                        //if (myresult == false)
                        //{
                        ///Print("face.TryGetPlane failed");
                        //    return myplane;
                        //}
                        NurbsSurface mysurfaceface = face.ToNurbsSurface();


                        foreach (ControlPoint cp in mysurfaceface.Points)
                        {
                            ptlist.Add(cp.Location);
                        }
                        Print("Here add a Brep");
                    }
                }

                else if (obj is Rhino.Geometry.Surface)
                {
                    Surface objsurface = (Surface)obj;
                    myresult = objsurface.TryGetPlane(out myplane);
                    //if (myresult == false)
                    //{
                    //    Print("surface.TryGetPlane failed");
                    //    return myplane;
                    //}
                    NurbsSurface mysurfaceface = objsurface.ToNurbsSurface();

                    foreach (ControlPoint cp in mysurfaceface.Points)
                    {
                        ptlist.Add(cp.Location);
                    }

                    Print("Here add a Surface");

                }

                else if (obj is Rhino.Geometry.Extrusion)
                {
                    Extrusion objextrusion = (Extrusion)obj;
                    myresult = objextrusion.TryGetPlane(out myplane);
                    //if (myresult == false)
                    //{
                    //    Print("surface.TryGetPlane failed");
                    //    return myplane;
                    //}

                    NurbsSurface mysurfaceface = objextrusion.ToNurbsSurface();

                    foreach (ControlPoint cp in mysurfaceface.Points)
                    {
                        ptlist.Add(cp.Location);
                    }
                    Print("Here add a Extrusion");
                }

                else if (obj is Rhino.Geometry.Mesh)
                {
                    Mesh objmesh = (Mesh)obj;
                    foreach (Point3d vert in objmesh.Vertices)
                    {
                        ptlist.Add(vert);

                    }
                    Print("Here add a mesh");
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
        public Box BoundingBoxPlane(List<GeometryBase> objs, Plane plane, bool accurate = true)
        {
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

            //if (ret_pts == true)                                ///can not
            //{
            //    Point3d[] corners = bbox.GetCorners();
            //    foreach (Point3d corner in corners)
            //    {
            //        corner.Transform(plane_to_world);
            //    }
            //    return corners;
            //}

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
            double inc = tot_ang / (divs-1);
            for(int i = 0; i < divs-1; i++)
            {
                plane.Rotate(inc, axis);
                out_plane.Add(new Rhino.Geometry.Plane(plane));
            }
            return out_plane;
        }

        public List<Plane> RotatePlaneArray3D(Plane view_plane, double tot_ang ,int divs)
        {
            //generate a 3D array of refinement planes (works with narrow angles)
            List<Plane> out_planes = new List<Plane>();
            //use RotatedPlaneArray to generate 'horizontal' left-right array (yaw)
            List<Plane> yaw_planes = RotatedPlaneArray(view_plane, tot_ang, divs, view_plane.ZAxis);
            foreach (Plane y_plane in yaw_planes)
            {
                //use RotatedPlaneArray to generate up-down 'tilt' array(roll)
                List<Plane> roll_planes = RotatedPlaneArray(y_plane,tot_ang,divs,y_plane.ZAxis);

                foreach (Plane r_plane in roll_planes)
                {
                    //use RotatedPlaneArray to generate up-down 'tilt' array(pitch)
                    List<Plane> pitch_planes = RotatedPlaneArray(r_plane, tot_ang, divs, r_plane.ZAxis);

                    foreach (Plane p_plane in pitch_planes)
                    {
                        out_planes.Add(p_plane);
                    }

                }
            }
        }

        //3D (non-planar) bounding box routine
        Min3DBoundingBox

        //this is the main 2D bb calculation search function
        PlanarMinBB

        //2D planar bounding rectangle routine
        MinBoundingRectanglePlane

        //used for planar bounding rectangle calculation
        BoxArea

        //Main 
        CombinedMinBBMulti
    }
}