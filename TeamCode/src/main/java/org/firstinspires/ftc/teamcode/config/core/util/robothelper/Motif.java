package org.firstinspires.ftc.teamcode.config.core.util.robothelper;

import android.content.res.Resources;

import java.util.Arrays;
import java.util.Optional;

public enum Motif {
    PPG(23), PGP(22), GPP(21);

    private final int id;

    Motif(int id) {
        this.id = id;
    }

    public int getId() {
        return id;
    }

    public static Motif getMotif(int id){
        Optional<Motif> res = Arrays.stream(Motif.values()).filter(motif -> motif.getId() == id).findFirst();
        if(res.isPresent()) return res.get();
        else throw new Resources.NotFoundException("ID does not match a Motif!");
    }
}