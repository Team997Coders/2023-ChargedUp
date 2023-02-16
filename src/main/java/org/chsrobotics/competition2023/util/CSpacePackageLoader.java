/**
Copyright 2023 FRC Team 997

This program is free software: 
you can redistribute it and/or modify it under the terms of the 
GNU General Public License as published by the Free Software Foundation, 
either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, 
but WITHOUT ANY WARRANTY; without even the implied warranty of 
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program. 
If not, see <https://www.gnu.org/licenses/>.
*/
package org.chsrobotics.competition2023.util;

import com.google.gson.Gson;
import java.io.File;
import java.io.FileWriter;
import java.nio.file.Files;
import org.chsrobotics.lib.math.geometry.Vector2D;
import org.chsrobotics.lib.trajectory.planning.ConfigurationSpace;
import org.chsrobotics.lib.util.NodeGraph;

public class CSpacePackageLoader {
    public static class CSpacePackage {
        public final ConfigurationSpace cSpace;
        public final NodeGraph<Vector2D> nodes;

        public CSpacePackage(ConfigurationSpace cSpace, NodeGraph<Vector2D> nodes) {
            this.cSpace = cSpace;
            this.nodes = nodes;
        }
    }

    public static CSpacePackage loadPackage(File cSpaceJSON) {
        try {
            return new Gson().fromJson(Files.readString(cSpaceJSON.toPath()), CSpacePackage.class);
        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }

    public static void writePackage(File writeTo, CSpacePackage pack) {
        try {
            FileWriter writer = new FileWriter(writeTo);

            writer.write(new Gson().toJson(pack));

            writer.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
