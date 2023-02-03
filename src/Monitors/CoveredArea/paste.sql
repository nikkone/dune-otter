
EXPLAIN QUERY PLAN 		select gid, distance(geometry, makepoint(554355.7,7022941.2,32632)) as dist from coverage where ROWID IN (
			SELECT ROWID FROM SpatialIndex
			WHERE f_table_name = 'coverage' and
			search_frame = BuildCircleMbr(554355.7,7022941.2, 670,32632)
		)



EXPLAIN QUERY PLAN update coverage set weight = min(weight+we*1.0, 1) from (
	select gid, 1/(1+exp(-(4.748444238767068 + distance(geometry, makepoint(554355.7,7022941.2,32632))*-0.011050835615990 ))) as we from coverage where ROWID IN (
			SELECT ROWID FROM SpatialIndex
			WHERE f_table_name = 'coverage' and
			search_frame = BuildCircleMbr(554355.7,7022941.2, 670,32632)
	)
) as a where coverage.gid = a.gid and we > 0.05

 EXPLAIN QUERY PLAN 
 update coverage set weight = min(weight+1/(1+exp(-(4.748444238767068 + distance(geometry, makepoint(554355.7,7022941.2,32632))*-0.011050835615990 )))*1.0, 1) where ROWID IN (
			SELECT ROWID FROM SpatialIndex
			WHERE f_table_name = 'coverage' and
			search_frame = BuildCircleMbr(554355.7,7022941.2, 670,32632)
	)

update coverage set weight = 0
select * from geometry_columns

			SELECT ROWID FROM SpatialIndex
			WHERE f_table_name = 'coverage' and
			f_geometry_column='geometry' and
			search_frame = BuildCircleMbr(554355.7,7022941.2, 670,32632)
			
			
SELECT COUNT(*) FROM location_ip2locationlitedb5
WHERE ROWID IN (

    SELECT pkid FROM idx_location_ip2locationlitedb5_coord_latlng
    WHERE xmin > 554355.7-335 AND xmax < 554355.7+335
      AND ymin > 7022941.2-335 AND ymax < 7022941.2+335
);

 EXPLAIN QUERY PLAN
select * from coverage where ROWID IN (
select pkid from idx_coverage_geometry WHERE
	xmin >= 554355.7-710 AND xmax <= 554355.7+710 AND
	ymin >= 7022941.2-710 AND ymax <= 7022941.2+710
)

 EXPLAIN QUERY PLAN
select * from coverage where ROWID IN (
			SELECT ROWID FROM SpatialIndex
			WHERE f_table_name = 'coverage' and
			f_geometry_column='geometry' and
			search_frame = BuildCircleMbr(554355.7,7022941.2, 670,32632)
)


CREATE INDEX idx_coverage_geometry_xmin_idx ON idx_coverage_geometry(xmin);
CREATE INDEX idx_coverage_geometry_xmax_idx ON idx_coverage_geometry(xmax);
CREATE INDEX idx_coverage_geometry_ymin_idx ON idx_coverage_geometry(ymin);
CREATE INDEX idx_coverage_geometry_ymin_idx ON idx_coverage_geometry(ymax);


 EXPLAIN QUERY PLAN 
 update coverage set weight = min(weight+1/(1+exp(-(4.748444238767068 + distance(geometry, makepoint(554355.7,7022941.2,32632))*-0.011050835615990 )))*1.0, 1) where ROWID IN (
select pkid from idx_coverage_geometry WHERE
	xmin >= 554355.7-710 AND xmax <= 554355.7+710 AND
	ymin >= 7022941.2-710 AND ymax <= 7022941.2+710
)



 EXPLAIN QUERY PLAN 
update coverage as s set weight = min(weight+we*1.0, 1) from (
select gid, 1/(1+exp(-(4.748444238767068 + dist*-0.011050835615990 ))) as we from (
	select gid, distance(c.geometry, makepoint(554355.7,7022941.2,32632)) as dist from (
		select gid, geometry from (
		select * from coverage where ROWID IN (SELECT ROWID FROM SpatialIndex
		WHERE f_table_name = 'coverage' and
		search_frame = BuildCircleMbr(554355.7,7022941.2, 750,32632))
		)
	) as c
)
) as a where s.gid = a.gid and we > 0.05

select * from rtree
