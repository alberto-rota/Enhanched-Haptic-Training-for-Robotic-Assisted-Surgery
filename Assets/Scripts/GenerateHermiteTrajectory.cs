// Copyright (c) 2022 Alberto Rota
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

using System.Collections.Generic;
using UnityEngine;

[ExecuteInEditMode, RequireComponent(typeof(LineRenderer))]
public class GenerateHermiteTrajectory : MonoBehaviour 
{
	// public Transform nodes;
	public List<GameObject> controlPoints = new List<GameObject>();
	public Color color = Color.green;
	[Range(0,1)]
	public float width = 0.002f;
	
	[Range(50,500)]
	public int numberOfPoints=100;
	LineRenderer lineRenderer;	

	void Start () 
	{
		controlPoints = new List<GameObject>();
        foreach (Transform child in gameObject.transform) {
            controlPoints.Add(child.gameObject);
        }
		// Debug.Log("NPoints");
		// Debug.Log(controlPoints.Count);
		lineRenderer = GetComponent<LineRenderer>();
		lineRenderer.startWidth = 0;
		lineRenderer.endWidth = 0;
		lineRenderer.useWorldSpace = true;
		if (lineRenderer.sharedMaterial ==  null)
			lineRenderer.sharedMaterial = Resources.Load<Material>("Materials/Trajectory");
	}
	
	void Update () 
	{
		if (null == lineRenderer || controlPoints == null 
			|| controlPoints.Count < 2)
   		{
      			return; // not enough points specified
   		}

		// update line renderer
		lineRenderer.startColor = color;
		lineRenderer.endColor = color;
   		lineRenderer.startWidth = width;
		lineRenderer.endWidth = width;
		if (numberOfPoints < 2)
   		{
      			numberOfPoints = 2;
   		}
		lineRenderer.positionCount = numberOfPoints * (controlPoints.Count - 1);

		// loop over segments of spline
		Vector3 p0, p1, m0, m1;

		for(int j = 0; j < controlPoints.Count - 1; j++)
		{
			// check control points
			if (controlPoints[j] == null || 
				controlPoints[j + 1] == null ||
				(j > 0 && controlPoints[j - 1] == null) ||
				(j < controlPoints.Count - 2 && controlPoints[j + 2] == null))
			{
				return;  
			}
			// determine control points of segment
			p0 = controlPoints[j].transform.position;
			p1 = controlPoints[j + 1].transform.position;
			
			if (j > 0) 
			{
				m0 = 0.5f * (controlPoints[j + 1].transform.position 
				- controlPoints[j - 1].transform.position);
			}
			else
			{
				m0 = controlPoints[j + 1].transform.position 
					- controlPoints[j].transform.position;
			}
			if (j < controlPoints.Count - 2)
			{
				m1 = 0.5f * (controlPoints[j + 2].transform.position 
					- controlPoints[j].transform.position);
			}
			else
			{
				m1 = controlPoints[j + 1].transform.position 
					- controlPoints[j].transform.position;
			}

			// set points of Hermite curve
			Vector3 position;
			float t;
			float pointStep = 1.0f / numberOfPoints;

			if (j == controlPoints.Count - 2)
			{
				pointStep = 1.0f / (numberOfPoints - 1.0f);
				// last point of last segment should reach p1
			}  
			for(int i = 0; i < numberOfPoints; i++) 
			{
				t = i * pointStep;
				position = (2.0f * t * t * t - 3.0f * t * t + 1.0f) * p0 
					+ (t * t * t - 2.0f * t * t + t) * m0 
					+ (-2.0f * t * t * t + 3.0f * t * t) * p1 
					+ (t * t * t - t * t) * m1;
				lineRenderer.SetPosition(i + j * numberOfPoints, 
					position);
			}
		}
	}
}