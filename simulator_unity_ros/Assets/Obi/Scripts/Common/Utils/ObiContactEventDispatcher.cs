using UnityEngine;
using UnityEngine.Events;
using System;
using System.Collections.Generic;

namespace Obi
{
    [RequireComponent(typeof(ObiSolver))]
    public class ObiContactEventDispatcher : MonoBehaviour
    {
        private ObiSolver solver;
        private Oni.Contact[] prevData;
        private int prevCount;
        private ContactComparer comparer;

        private class ContactComparer : IComparer<Oni.Contact>
        {
            ObiSolver solver;

            public ContactComparer(ObiSolver solver)
            {
                this.solver = solver;
            }

            public int Compare(Oni.Contact x, Oni.Contact y)
            {
                return CompareByRef(ref x, ref y, solver);
            }
        }

        private static int CompareByRef(ref Oni.Contact a, ref Oni.Contact b, ObiSolver solver)
        {
            if (a.other == b.other)
            {
                int hashA = solver.particleToActor[a.particle].actor.GetInstanceID();
                int hashB = solver.particleToActor[b.particle].actor.GetInstanceID();
                return hashA.CompareTo(hashB);
            }
            return a.other.CompareTo(b.other);
        }

        [System.Serializable]
        public class ContactCallback : UnityEvent<ObiSolver, Oni.Contact> { }

        public float distanceThreshold = 0.01f;
        public ContactCallback onContactEnter = new ContactCallback();
        public ContactCallback onContactStay = new ContactCallback();
        public ContactCallback onContactExit = new ContactCallback();

        void Awake()
        {
            solver = GetComponent<ObiSolver>();
            comparer = new ContactComparer(solver);
            prevData = new Oni.Contact[0];
        }

        void OnEnable()
        {
            solver.OnCollision += Solver_OnCollision;
        }

        void OnDisable()
        {
            solver.OnCollision -= Solver_OnCollision;
        }

        private int FilterOutDistantContacts(Oni.Contact[] data, int count)
        {
            int filteredCount = count;

            // simply iterate trough all contacts,
            // moving the ones above the threshold to the end of the array:
            for (int i = count -1 ; i >= 0; --i)
                if (data[i].distance > distanceThreshold)
                    ObiUtils.Swap(ref data[i], ref data[--filteredCount]);

            return filteredCount;
        }

        private int RemoveDuplicates(Oni.Contact[] data, int count)
        {
            if (count == 0)
                return 0;

            // assuming the array is sorted, iterate trough the array
            // replacing duplicates by the first contact that's different:
            int i = 0, r = 0;
            while (++i != count)
                if (CompareByRef(ref data[i], ref data[r], solver) != 0 && ++r != i)
                    data[r] = data[i];

            return ++r;
        }

        private void InvokeCallbacks(Oni.Contact[] data, int count)
        {
            int a = 0, b = 0;
            int lengthA = count, lengthB = prevCount;

            // while we haven't reached the end of either array:
            while (a < lengthA && b < lengthB)
            {
                // compare both contacts: 
                int compare = CompareByRef(ref data[a], ref prevData[b], solver);

                // call the appropiate event depending on the comparison result:
                if (compare < 0)
                    onContactEnter.Invoke(solver, data[a++]);
                else if (compare > 0)
                    onContactExit.Invoke(solver, prevData[b++]);
                else
                {
                    onContactStay.Invoke(solver, data[a++]); b++;
                }
            }

            // finish iterating trough both lists:
            while (a < lengthA)
                onContactEnter.Invoke(solver, data[a++]);

            while (b < lengthB)
                onContactExit.Invoke(solver, prevData[b++]);
        }

        void Solver_OnCollision(object sender, ObiSolver.ObiCollisionEventArgs args)
        {
            // here we access the internal backing array (Data) directly,
            // instead of using the accessor property. This slightly improves performance.
            // note: the backing array length is the lists' capacity, so we
            // need to use args.contacts.Count to get the actual number of contacts.

            // skip all contacts above the distance threshold by moving them to the end of the array:
            int filteredCount = FilterOutDistantContacts(args.contacts.Data, args.contacts.Count);

            // sort the remaining contacts by collider, then by actor:
            Array.Sort(args.contacts.Data, 0, filteredCount, comparer);

            // remove duplicates:
            filteredCount = RemoveDuplicates(args.contacts.Data, filteredCount);

            // zip trough the current and previous contact lists once, invoking events when appropiate.
            InvokeCallbacks(args.contacts.Data, filteredCount);

            // store current contact list/count for next frame.
            // could get better performance by double buffering instead of copying:
            if (filteredCount > prevData.Length) 
                Array.Resize(ref prevData, filteredCount);
            Array.Copy(args.contacts.Data, prevData, filteredCount);

            prevCount = filteredCount;
        }

    }
}