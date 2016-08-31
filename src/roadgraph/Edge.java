package roadgraph;

import geography.GeographicPoint;

/**
 * Created by Broulaye on 7/14/2016.
 */
public class Edge {

    private String roadName;
    private String roadType;
    private double length;
    private GeographicPoint Start;
    private GeographicPoint End;

    public Edge(String roadName, String roadType, double length, GeographicPoint Start, GeographicPoint End) {
        this.roadName = roadName;
        this.roadType = roadType;
        this.length = length;
        this.Start = Start;
        this.End = End;
    }

    public GeographicPoint getEnd() {
        return End;
    }

    public void setEnd(GeographicPoint end) {
        End = end;
    }

    public String getRoadName() {
        return roadName;
    }

    public void setRoadName(String roadName) {
        this.roadName = roadName;
    }

    public String getRoadType() {
        return roadType;
    }

    public void setRoadType(String roadType) {
        this.roadType = roadType;
    }

    public double getLength() {
        return length;
    }

    public void setLength(double length) {
        this.length = length;
    }

    public GeographicPoint getStart() {
        return Start;
    }

    public void setStart(GeographicPoint start) {
        this.Start = start;
    }
}
