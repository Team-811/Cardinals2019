package frc.robot.lib.geometry;

import frc.robot.lib.util.CSVWritable;
import frc.robot.lib.util.Interpolable;

public interface State<S> extends Interpolable<S>, CSVWritable {
    double distance(final S other);

    boolean equals(final Object other);

    String toString();

    String toCSV();
}
