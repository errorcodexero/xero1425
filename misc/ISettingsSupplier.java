package org.xero1425.misc;

import java.util.List;

public interface ISettingsSupplier {
    boolean isDefined(String name)  ;
    SettingsValue get(String name) throws MissingParameterException ;
    SettingsValue getOrNull(String name) ;
    List<String> getAllKeys(String path) ;
}
