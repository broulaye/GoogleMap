package roadgraph;

import geography.GeographicPoint;

import java.util.LinkedList;
import java.util.List;

/**
 * Created by Broulaye on 7/18/2016.
 */
public class MapNode implements Comparable {
    private GeographicPoint location;
    private List<Edge> Edges;
    private double currDist;
    private double predictedDist;

    public MapNode(GeographicPoint location, List<Edge> edges, double dist, double pDist) {
        this.location = location;
        Edges = edges;
        currDist = dist;
        predictedDist = pDist;
    }

    public MapNode(GeographicPoint location) {
        this.location = location;
        Edges = new LinkedList<Edge>();
        currDist = Double.POSITIVE_INFINITY;
        predictedDist = Double.POSITIVE_INFINITY;
    }

    public double getPredictedDist() {
        return predictedDist;
    }

    public void setPredictedDist(double predictedDist) {
        this.predictedDist = predictedDist;
    }

    public GeographicPoint getLocation() {
        return location;
    }

    public double getCurrDist() {
        return currDist;
    }

    public void setCurrDist(double currDist) {
        this.currDist = currDist;
    }

    public void setLocation(GeographicPoint location) {
        this.location = location;
    }

    public List<Edge> getEdges() {
        return Edges;
    }

    public void setEdges(List<Edge> edges) {
        Edges = edges;
    }

    @Override
    public int compareTo(Object o) {

        MapNode x = (MapNode)o;

        if(this.predictedDist > x.predictedDist) {
            return 1;
        }
        else if(this.predictedDist < x.predictedDist) {
            return -1;
        }
        else
            return 0;
    }

    public double DistTo(GeographicPoint goal) {

        return this.location.distance(goal);
    }
}
