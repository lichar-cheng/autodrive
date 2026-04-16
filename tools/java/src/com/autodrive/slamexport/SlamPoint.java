package com.autodrive.slamexport;

public final class SlamPoint {
    private final float x;
    private final float y;
    private final float intensity;

    public SlamPoint(float x, float y, float intensity) {
        this.x = x;
        this.y = y;
        this.intensity = intensity;
    }

    public float getX() {
        return x;
    }

    public float getY() {
        return y;
    }

    public float getIntensity() {
        return intensity;
    }
}
