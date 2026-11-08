package org.firstinspires.ftc.teamcode.config.core.util.robothelper;

import java.util.function.Predicate;

public enum ArtifactMatch {
    NONE((a) -> a==Artifact.NONE), PURPLE((a) -> a == Artifact.PURPLE), GREEN((a) -> a == Artifact.GREEN), ANY((a) -> a != Artifact.NONE);
    private final Predicate<Artifact> m_predicate;
    ArtifactMatch(Predicate<Artifact> predicate){
        this.m_predicate = predicate;
    }
    public Predicate<Artifact> getPredicate(){ return this.m_predicate; }
}
