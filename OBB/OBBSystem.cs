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
                    Curve objcurve = (Curve) obj;
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

                else if(obj is Rhino.Geometry.Point)
                {
                    Point objpoint = (Point)obj;
                    ptlist.Add(objpoint.Location);
                    Print("Here add a Point");
                }

                else if(obj is Rhino.Geometry.PointCloud)
                {
                    PointCloud objpointcloud = (PointCloud)obj;
                    Point3d[] ptarray = objpointcloud.GetPoints();
                    foreach (Point3d i in ptarray)
                    {
                        ptlist.Add(i);
                    }
                    Print("Here add a PointCloud");
                }

                else if(obj is Rhino.Geometry.Brep)
                {
                    Brep objbrep = (Brep)obj;   
                    foreach(BrepFace face in objbrep.Faces)
                    {
                        myresult = face.TryGetPlane(out myplane);

                        //if (myresult == false)
                        //{
                            ///Print("face.TryGetPlane failed");
                        //    return myplane;
                        //}
                        NurbsSurface mysurfaceface = face.ToNurbsSurface();


                        foreach(ControlPoint cp in mysurfaceface.Points)
                        {
                            ptlist.Add(cp.Location);
                        }
                        Print("Here add a Brep");
                    }
                }

                else if(obj is Rhino.Geometry.Surface)
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

                     foreach(ControlPoint cp in mysurfaceface.Points)
                        {
                            ptlist.Add(cp.Location);
                        }
                    Print("Here add a Extrusion");
                }

                else if (obj is Rhino.Geometry.Mesh)
                {
                    Mesh objmesh = (Mesh)obj;
                    foreach(Point3d vert in objmesh.Vertices)
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
    }