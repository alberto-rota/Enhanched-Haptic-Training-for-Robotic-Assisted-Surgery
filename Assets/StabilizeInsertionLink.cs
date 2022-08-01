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


// If you are reading this script, I have to admit in full disclosure that I have no idea of what I'm doing.
// I'm sorry.
// I have no idea why this works.
// I have no idea what causes the InsertionLink to drift.
// I have no idea why the drift is not constant on the three axis
// I have no idea why compensating for the drift on the Z axis fixing the problem, while the object clearly moves along the X axis.

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using UnityEngine.SceneManagement;


public class StabilizeInsertionLink : MonoBehaviour
{
    [Range(-0.00001f,0)]
    public float DRIFT = -0.000001f;

    void Update()
    {

        gameObject.transform.localPosition = new Vector3(gameObject.transform.localPosition.x,gameObject.transform.localPosition.y, gameObject.transform.localPosition.z+DRIFT);
    }
}
